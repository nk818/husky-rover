#!/usr/bin/env python3

import sys
import os

# Add current directory to Python path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, JointState, LaserScan
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from flask import Flask, render_template, jsonify, send_file, request
from flask_cors import CORS
from waitress import serve
import threading
import math
import json
import csv
from datetime import datetime
from collections import deque
import subprocess

try:
    from anomaly_detector import HuskyAnomalyDetector
    ANOMALY_DETECTION_AVAILABLE = True
except ImportError as e:
    print(f"⚠️  Anomaly detection not available: {e}")
    ANOMALY_DETECTION_AVAILABLE = False
    HuskyAnomalyDetector = None

app = Flask(__name__)
CORS(app)

# Global anomaly detector
if ANOMALY_DETECTION_AVAILABLE:
    anomaly_detector = HuskyAnomalyDetector()
else:
    anomaly_detector = None

class HuskyDashboard(Node):
    def __init__(self):
        super().__init__('husky_dashboard')
        
        # Initialize data storage
        self.data = {
            "imu": {
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                "angular_vel": {"x": 0.0, "y": 0.0, "z": 0.0},
                "linear_acc": {"x": 0.0, "y": 0.0, "z": 0.0}
            },
            "gps": {"lat": 0.0, "lon": 0.0, "alt": 0.0},
            "odom": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "velocity": {"linear": 0.0, "angular": 0.0}
            },
            "cmd_vel": {"linear": 0.0, "angular": 0.0},
            "scan": {
                "range_min": 0.0,
                "range_max": 0.0,
                "avg_range": 0.0,
                "min_range": 0.0,
                "max_range": 0.0,
                "num_readings": 0,
                "obstacles_detected": 0
            },
            "joints": [],
            "diagnostics": [],
            "last_update": datetime.now().isoformat()
        }
        
        # Historical data for graphs (keep last 100 points)
        self.history = {
            "timestamps": deque(maxlen=100),
            "imu_roll": deque(maxlen=100),
            "imu_pitch": deque(maxlen=100),
            "imu_yaw": deque(maxlen=100),
            "velocity": deque(maxlen=100),
            "acceleration_x": deque(maxlen=100),
            "acceleration_y": deque(maxlen=100),
            "acceleration_z": deque(maxlen=100),
            "scan_avg_range": deque(maxlen=100),
            "scan_min_range": deque(maxlen=100),
        }
        
        # Data logging
        self.logging_enabled = False
        self.log_data = []
        self.log_file = None
        
        # Alerts configuration
        self.alerts = []
        self.alert_thresholds = {
            "velocity": 2.0,
            "acceleration": 15.0,
            "angular_velocity": 3.0
        }
        
        # Anomaly detection
        self.anomaly_alerts = []
        self.current_anomaly_status = {"is_anomaly": False, "score": 0.0, "features": {}}
        
        # Bag playback control
        self.bag_process = None
        self.bag_file_path = None
        
        # Create subscribers
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/data', self.gps_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.diag_sub = self.create_subscription(DiagnosticArray, '/diagnostics', self.diagnostics_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.get_logger().info('Husky Dashboard Node Started')
        
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    
    def check_alerts(self):
        """Check if any values exceed thresholds"""
        current_time = datetime.now().strftime("%H:%M:%S")
        
        # Check velocity
        vel = abs(self.data["odom"]["velocity"]["linear"])
        if vel > self.alert_thresholds["velocity"]:
            self.alerts.append(f"[{current_time}] HIGH VELOCITY: {vel:.2f} m/s")
        
        # Check acceleration
        acc_x = abs(self.data["imu"]["linear_acc"]["x"])
        acc_y = abs(self.data["imu"]["linear_acc"]["y"])
        acc_z = abs(self.data["imu"]["linear_acc"]["z"])
        max_acc = max(acc_x, acc_y, acc_z)
        if max_acc > self.alert_thresholds["acceleration"]:
            self.alerts.append(f"[{current_time}] HIGH ACCELERATION: {max_acc:.2f} m/s²")
        
        # Check angular velocity
        ang_vel = abs(self.data["imu"]["angular_vel"]["z"])
        if ang_vel > self.alert_thresholds["angular_velocity"]:
            self.alerts.append(f"[{current_time}] HIGH ANGULAR VELOCITY: {ang_vel:.2f} rad/s")
        
        # Keep only last 50 alerts
        if len(self.alerts) > 50:
            self.alerts = self.alerts[-50:]
    
    def check_anomaly(self):
        """Check current data for anomalies using ML model"""
        global anomaly_detector
        
        if not ANOMALY_DETECTION_AVAILABLE or not anomaly_detector or not anomaly_detector.is_trained:
            return
        
        try:
            # Create data entry for anomaly detection
            data_entry = {
                "imu_roll": self.data["imu"]["roll"],
                "imu_pitch": self.data["imu"]["pitch"],
                "imu_yaw": self.data["imu"]["yaw"],
                "imu_ang_vel_x": self.data["imu"]["angular_vel"]["x"],
                "imu_ang_vel_y": self.data["imu"]["angular_vel"]["y"],
                "imu_ang_vel_z": self.data["imu"]["angular_vel"]["z"],
                "imu_lin_acc_x": self.data["imu"]["linear_acc"]["x"],
                "imu_lin_acc_y": self.data["imu"]["linear_acc"]["y"],
                "imu_lin_acc_z": self.data["imu"]["linear_acc"]["z"],
                "odom_vel_linear": self.data["odom"]["velocity"]["linear"],
                "odom_vel_angular": self.data["odom"]["velocity"]["angular"],
                "cmd_vel_linear": self.data["cmd_vel"]["linear"],
                "cmd_vel_angular": self.data["cmd_vel"]["angular"]
            }
            
            is_anomaly, score, features = anomaly_detector.predict(data_entry)
            
            self.current_anomaly_status = {
                "is_anomaly": is_anomaly,
                "score": score,
                "features": features
            }
            
            if is_anomaly:
                current_time = datetime.now().strftime("%H:%M:%S")
                self.anomaly_alerts.append(
                    f"[{current_time}] ANOMALY DETECTED: Score={score:.3f}"
                )
                
                # Keep only last 50 anomaly alerts
                if len(self.anomaly_alerts) > 50:
                    self.anomaly_alerts = self.anomaly_alerts[-50:]
        
        except Exception as e:
            self.get_logger().warning(f"Anomaly detection error: {str(e)}")
    
    def log_current_data(self):
        """Log current data point"""
        if self.logging_enabled:
            log_entry = {
                "timestamp": datetime.now().isoformat(),
                "imu_roll": self.data["imu"]["roll"],
                "imu_pitch": self.data["imu"]["pitch"],
                "imu_yaw": self.data["imu"]["yaw"],
                "imu_ang_vel_x": self.data["imu"]["angular_vel"]["x"],
                "imu_ang_vel_y": self.data["imu"]["angular_vel"]["y"],
                "imu_ang_vel_z": self.data["imu"]["angular_vel"]["z"],
                "imu_lin_acc_x": self.data["imu"]["linear_acc"]["x"],
                "imu_lin_acc_y": self.data["imu"]["linear_acc"]["y"],
                "imu_lin_acc_z": self.data["imu"]["linear_acc"]["z"],
                "gps_lat": self.data["gps"]["lat"],
                "gps_lon": self.data["gps"]["lon"],
                "gps_alt": self.data["gps"]["alt"],
                "odom_pos_x": self.data["odom"]["position"]["x"],
                "odom_pos_y": self.data["odom"]["position"]["y"],
                "odom_pos_z": self.data["odom"]["position"]["z"],
                "odom_vel_linear": self.data["odom"]["velocity"]["linear"],
                "odom_vel_angular": self.data["odom"]["velocity"]["angular"],
                "cmd_vel_linear": self.data["cmd_vel"]["linear"],
                "cmd_vel_angular": self.data["cmd_vel"]["angular"],
                "scan_avg_range": self.data["scan"]["avg_range"],
                "scan_min_range": self.data["scan"]["min_range"],
                "scan_max_range": self.data["scan"]["max_range"],
                "scan_num_readings": self.data["scan"]["num_readings"],
                "scan_obstacles_detected": self.data["scan"]["obstacles_detected"]
            }
            self.log_data.append(log_entry)
    
    def imu_callback(self, msg):
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.orientation.x, msg.orientation.y, 
            msg.orientation.z, msg.orientation.w
        )
        self.data["imu"]["roll"] = round(roll, 3)
        self.data["imu"]["pitch"] = round(pitch, 3)
        self.data["imu"]["yaw"] = round(yaw, 3)
        self.data["imu"]["angular_vel"]["x"] = round(msg.angular_velocity.x, 3)
        self.data["imu"]["angular_vel"]["y"] = round(msg.angular_velocity.y, 3)
        self.data["imu"]["angular_vel"]["z"] = round(msg.angular_velocity.z, 3)
        self.data["imu"]["linear_acc"]["x"] = round(msg.linear_acceleration.x, 3)
        self.data["imu"]["linear_acc"]["y"] = round(msg.linear_acceleration.y, 3)
        self.data["imu"]["linear_acc"]["z"] = round(msg.linear_acceleration.z, 3)
        self.data["last_update"] = datetime.now().isoformat()
        
        # Update history
        current_time = datetime.now().timestamp()
        self.history["timestamps"].append(current_time)
        self.history["imu_roll"].append(roll)
        self.history["imu_pitch"].append(pitch)
        self.history["imu_yaw"].append(yaw)
        self.history["acceleration_x"].append(msg.linear_acceleration.x)
        self.history["acceleration_y"].append(msg.linear_acceleration.y)
        self.history["acceleration_z"].append(msg.linear_acceleration.z)
        
        self.check_alerts()
        self.check_anomaly()
        self.log_current_data()
    
    def gps_callback(self, msg):
        self.data["gps"]["lat"] = round(msg.latitude, 6)
        self.data["gps"]["lon"] = round(msg.longitude, 6)
        self.data["gps"]["alt"] = round(msg.altitude, 2)
        self.data["last_update"] = datetime.now().isoformat()
        self.log_current_data()
    
    def odom_callback(self, msg):
        self.data["odom"]["position"]["x"] = round(msg.pose.pose.position.x, 3)
        self.data["odom"]["position"]["y"] = round(msg.pose.pose.position.y, 3)
        self.data["odom"]["position"]["z"] = round(msg.pose.pose.position.z, 3)
        self.data["odom"]["velocity"]["linear"] = round(msg.twist.twist.linear.x, 3)
        self.data["odom"]["velocity"]["angular"] = round(msg.twist.twist.angular.z, 3)
        self.data["last_update"] = datetime.now().isoformat()
        
        # Update velocity history
        self.history["velocity"].append(msg.twist.twist.linear.x)
        
        self.log_current_data()
    
    def joint_callback(self, msg):
        joints = []
        for i, name in enumerate(msg.name):
            joint = {
                "name": name,
                "position": round(msg.position[i], 3) if i < len(msg.position) else 0.0,
                "velocity": round(msg.velocity[i], 3) if i < len(msg.velocity) else 0.0
            }
            joints.append(joint)
        self.data["joints"] = joints
        self.data["last_update"] = datetime.now().isoformat()
    
    def cmd_vel_callback(self, msg):
        self.data["cmd_vel"]["linear"] = round(msg.linear.x, 3)
        self.data["cmd_vel"]["angular"] = round(msg.angular.z, 3)
        self.data["last_update"] = datetime.now().isoformat()
    
    def diagnostics_callback(self, msg):
        self.data["diagnostics"] = [status.name for status in msg.status]
        self.data["last_update"] = datetime.now().isoformat()
    
    def scan_callback(self, msg):
        """Process laser scan data"""
        import numpy as np
        
        # Filter out invalid readings (inf, nan)
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max) & np.isfinite(ranges)]
        
        if len(valid_ranges) > 0:
            avg_range = float(np.mean(valid_ranges))
            min_range = float(np.min(valid_ranges))
            max_range = float(np.max(valid_ranges))
            
            # Count obstacles (readings closer than 2 meters)
            obstacles = np.sum(valid_ranges < 2.0)
            
            self.data["scan"]["range_min"] = round(msg.range_min, 3)
            self.data["scan"]["range_max"] = round(msg.range_max, 3)
            self.data["scan"]["avg_range"] = round(avg_range, 3)
            self.data["scan"]["min_range"] = round(min_range, 3)
            self.data["scan"]["max_range"] = round(max_range, 3)
            self.data["scan"]["num_readings"] = len(valid_ranges)
            self.data["scan"]["obstacles_detected"] = int(obstacles)
            
            # Update history
            self.history["scan_avg_range"].append(avg_range)
            self.history["scan_min_range"].append(min_range)
        
        self.data["last_update"] = datetime.now().isoformat()
        self.log_current_data()

# Global node instance
dashboard_node = None

@app.route('/')
def index():
    return render_template('dashboard.html')

@app.route('/api/data')
def get_data():
    if dashboard_node:
        return jsonify(dashboard_node.data)
    return jsonify({"error": "Node not initialized"})

@app.route('/api/history')
def get_history():
    if dashboard_node:
        history_data = {
            "timestamps": list(dashboard_node.history["timestamps"]),
            "imu_roll": list(dashboard_node.history["imu_roll"]),
            "imu_pitch": list(dashboard_node.history["imu_pitch"]),
            "imu_yaw": list(dashboard_node.history["imu_yaw"]),
            "velocity": list(dashboard_node.history["velocity"]),
            "acceleration_x": list(dashboard_node.history["acceleration_x"]),
            "acceleration_y": list(dashboard_node.history["acceleration_y"]),
            "acceleration_z": list(dashboard_node.history["acceleration_z"]),
            "scan_avg_range": list(dashboard_node.history["scan_avg_range"]),
            "scan_min_range": list(dashboard_node.history["scan_min_range"])
        }
        return jsonify(history_data)
    return jsonify({"error": "Node not initialized"})

@app.route('/api/alerts')
def get_alerts():
    if dashboard_node:
        return jsonify({"alerts": dashboard_node.alerts})
    return jsonify({"alerts": []})

@app.route('/api/logging/start', methods=['POST'])
def start_logging():
    if dashboard_node:
        dashboard_node.logging_enabled = True
        dashboard_node.log_data = []
        return jsonify({"status": "Logging started"})
    return jsonify({"error": "Node not initialized"}), 500

@app.route('/api/logging/stop', methods=['POST'])
def stop_logging():
    if dashboard_node:
        dashboard_node.logging_enabled = False
        return jsonify({"status": "Logging stopped", "records": len(dashboard_node.log_data)})
    return jsonify({"error": "Node not initialized"}), 500

@app.route('/api/logging/status')
def logging_status():
    if dashboard_node:
        return jsonify({
            "enabled": dashboard_node.logging_enabled,
            "records": len(dashboard_node.log_data)
        })
    return jsonify({"error": "Node not initialized"}), 500

@app.route('/api/export/csv')
def export_csv():
    if dashboard_node and dashboard_node.log_data:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"husky_data_{timestamp}.csv"
        filepath = os.path.join('/tmp', filename)
        
        # Write CSV
        with open(filepath, 'w', newline='') as csvfile:
            if dashboard_node.log_data:
                fieldnames = dashboard_node.log_data[0].keys()
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(dashboard_node.log_data)
        
        return send_file(filepath, as_attachment=True, download_name=filename)
    return jsonify({"error": "No data to export"}), 400

@app.route('/api/alerts/configure', methods=['POST'])
def configure_alerts():
    if dashboard_node:
        data = request.json
        if 'velocity' in data:
            dashboard_node.alert_thresholds['velocity'] = float(data['velocity'])
        if 'acceleration' in data:
            dashboard_node.alert_thresholds['acceleration'] = float(data['acceleration'])
        if 'angular_velocity' in data:
            dashboard_node.alert_thresholds['angular_velocity'] = float(data['angular_velocity'])
        return jsonify({"status": "Thresholds updated", "thresholds": dashboard_node.alert_thresholds})
    return jsonify({"error": "Node not initialized"}), 500

@app.route('/api/alerts/thresholds')
def get_thresholds():
    if dashboard_node:
        return jsonify(dashboard_node.alert_thresholds)
    return jsonify({"error": "Node not initialized"}), 500

@app.route('/api/alerts/clear', methods=['POST'])
def clear_alerts():
    if dashboard_node:
        dashboard_node.alerts = []
        return jsonify({"status": "Alerts cleared"})
    return jsonify({"error": "Node not initialized"}), 500

@app.route('/api/anomaly/train', methods=['POST'])
def train_anomaly_model():
    """Train anomaly detection model on logged data"""
    global anomaly_detector
    
    data = request.json
    csv_file = data.get('csv_file')
    contamination = float(data.get('contamination', 0.1))
    
    if not csv_file or not os.path.exists(csv_file):
        return jsonify({"error": "CSV file not found"}), 400
    
    try:
        anomaly_detector.train(csv_file, contamination=contamination)
        anomaly_detector.save_model()
        return jsonify({
            "status": "Model trained successfully",
            "features": anomaly_detector.feature_names
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/api/anomaly/load', methods=['POST'])
def load_anomaly_model():
    """Load a trained anomaly detection model"""
    global anomaly_detector
    
    try:
        anomaly_detector.load_model()
        return jsonify({
            "status": "Model loaded successfully",
            "features": anomaly_detector.feature_names
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/api/anomaly/status')
def get_anomaly_status():
    """Get current anomaly detection status"""
    if not ANOMALY_DETECTION_AVAILABLE or not anomaly_detector:
        return jsonify({"error": "Anomaly detection not available"}), 503
    
    if dashboard_node:
        return jsonify({
            "model_trained": anomaly_detector.is_trained,
            "current_status": dashboard_node.current_anomaly_status,
            "recent_anomalies": dashboard_node.anomaly_alerts[-10:] if dashboard_node.anomaly_alerts else []
        })
    return jsonify({"error": "Node not initialized"}), 500

@app.route('/api/anomaly/alerts')
def get_anomaly_alerts():
    """Get all anomaly alerts"""
    if dashboard_node:
        return jsonify({"alerts": dashboard_node.anomaly_alerts})
    return jsonify({"alerts": []})

def ros_spin():
    rclpy.spin(dashboard_node)

def create_html_template():
    """Create the HTML template file"""
    
    template_dir = 'templates'
    if not os.path.exists(template_dir):
        os.makedirs(template_dir)
    
    html_content = '''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Husky Robot Dashboard</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%);
            color: #fff;
            padding: 20px;
        }
        
        .container {
            max-width: 1600px;
            margin: 0 auto;
        }
        
        h1 {
            text-align: center;
            margin-bottom: 20px;
            font-size: 2.5em;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        
        .controls-bar {
            background: rgba(255, 255, 255, 0.1);
            backdrop-filter: blur(10px);
            border-radius: 15px;
            padding: 15px;
            margin-bottom: 20px;
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
            align-items: center;
            box-shadow: 0 8px 32px 0 rgba(31, 38, 135, 0.37);
        }
        
        .btn {
            padding: 10px 20px;
            border: none;
            border-radius: 8px;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.3s;
            font-size: 14px;
        }
        
        .btn-primary {
            background: #00ff00;
            color: #000;
        }
        
        .btn-danger {
            background: #ff4444;
            color: #fff;
        }
        
        .btn-info {
            background: #4444ff;
            color: #fff;
        }
        
        .btn-warning {
            background: #ffaa00;
            color: #000;
        }
        
        .btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.3);
        }
        
        .btn:disabled {
            opacity: 0.5;
            cursor: not-allowed;
        }
        
        .status-badge {
            padding: 8px 15px;
            border-radius: 20px;
            font-size: 12px;
            font-weight: bold;
            margin-left: auto;
        }
        
        .status-recording {
            background: #ff4444;
            animation: pulse 1.5s infinite;
        }
        
        .status-idle {
            background: rgba(255, 255, 255, 0.2);
        }
        
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.6; }
        }
        
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
            gap: 20px;
            margin-bottom: 20px;
        }
        
        .card {
            background: rgba(255, 255, 255, 0.1);
            backdrop-filter: blur(10px);
            border-radius: 15px;
            padding: 20px;
            box-shadow: 0 8px 32px 0 rgba(31, 38, 135, 0.37);
            border: 1px solid rgba(255, 255, 255, 0.18);
        }
        
        .card-header {
            font-size: 1.5em;
            margin-bottom: 15px;
            padding-bottom: 10px;
            border-bottom: 2px solid rgba(255, 255, 255, 0.3);
            display: flex;
            align-items: center;
        }
        
        .card-header .icon {
            margin-right: 10px;
            font-size: 1.2em;
        }
        
        .data-row {
            display: flex;
            justify-content: space-between;
            padding: 8px 0;
            border-bottom: 1px solid rgba(255, 255, 255, 0.1);
        }
        
        .data-label {
            color: #a0d8ff;
            font-weight: 600;
        }
        
        .data-value {
            color: #fff;
            font-family: 'Courier New', monospace;
            font-weight: bold;
        }
        
        .chart-container {
            position: relative;
            height: 300px;
            margin-top: 10px;
        }
        
        .full-width {
            grid-column: 1 / -1;
        }
        
        .alerts-container {
            max-height: 200px;
            overflow-y: auto;
            margin-top: 10px;
        }
        
        .alert-item {
            padding: 8px;
            margin: 5px 0;
            background: rgba(255, 68, 68, 0.2);
            border-left: 3px solid #ff4444;
            border-radius: 5px;
            font-size: 0.9em;
        }
        
        .alert-config {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
            margin-top: 10px;
        }
        
        .alert-config input {
            padding: 8px;
            border-radius: 5px;
            border: 1px solid rgba(255,255,255,0.3);
            background: rgba(255,255,255,0.1);
            color: #fff;
        }
        
        .joints-table {
            width: 100%;
            margin-top: 10px;
        }
        
        .joints-table td {
            padding: 5px;
            font-size: 0.9em;
        }
        
        .diagnostics-list {
            max-height: 150px;
            overflow-y: auto;
            margin-top: 10px;
        }
        
        .diagnostic-item {
            padding: 5px;
            margin: 2px 0;
            background: rgba(0, 255, 0, 0.1);
            border-left: 3px solid #00ff00;
            font-size: 0.9em;
        }
        
        .last-update {
            text-align: center;
            margin-top: 20px;
            opacity: 0.7;
            font-size: 0.9em;
        }
        
        .record-count {
            padding: 8px 15px;
            background: rgba(255,255,255,0.2);
            border-radius: 20px;
            font-size: 12px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 HUSKY ROBOT SENSOR DASHBOARD</h1>
        
        <!-- Control Bar -->
        <div class="controls-bar">
            <button class="btn btn-primary" id="startLoggingBtn" onclick="startLogging()">
                ▶️ Start Logging
            </button>
            <button class="btn btn-danger" id="stopLoggingBtn" onclick="stopLogging()" disabled>
                ⏹️ Stop Logging
            </button>
            <button class="btn btn-info" onclick="exportCSV()">
                📥 Export CSV
            </button>
            <button class="btn btn-warning" onclick="clearAlerts()">
                🔔 Clear Alerts
            </button>
            <span class="record-count" id="recordCount">Records: 0</span>
            <span class="status-badge status-idle" id="statusBadge">⚪ IDLE</span>
        </div>
        
        <div class="grid">
            <!-- IMU Card -->
            <div class="card">
                <div class="card-header">
                    <span class="icon">📊</span>
                    <span>IMU Data</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Roll:</span>
                    <span class="data-value" id="imu-roll">0.000°</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Pitch:</span>
                    <span class="data-value" id="imu-pitch">0.000°</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Yaw:</span>
                    <span class="data-value" id="imu-yaw">0.000°</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Angular Vel X:</span>
                    <span class="data-value" id="imu-ang-x">0.000 rad/s</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Angular Vel Y:</span>
                    <span class="data-value" id="imu-ang-y">0.000 rad/s</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Angular Vel Z:</span>
                    <span class="data-value" id="imu-ang-z">0.000 rad/s</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Linear Acc X:</span>
                    <span class="data-value" id="imu-acc-x">0.000 m/s²</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Linear Acc Y:</span>
                    <span class="data-value" id="imu-acc-y">0.000 m/s²</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Linear Acc Z:</span>
                    <span class="data-value" id="imu-acc-z">0.000 m/s²</span>
                </div>
            </div>
            
            <!-- GPS Card -->
            <div class="card">
                <div class="card-header">
                    <span class="icon">🌍</span>
                    <span>GPS Data</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Latitude:</span>
                    <span class="data-value" id="gps-lat">0.000000°</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Longitude:</span>
                    <span class="data-value" id="gps-lon">0.000000°</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Altitude:</span>
                    <span class="data-value" id="gps-alt">0.00 m</span>
                </div>
            </div>
            
            <!-- Odometry Card -->
            <div class="card">
                <div class="card-header">
                    <span class="icon">📍</span>
                    <span>Odometry</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Position X:</span>
                    <span class="data-value" id="odom-x">0.000 m</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Position Y:</span>
                    <span class="data-value" id="odom-y">0.000 m</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Position Z:</span>
                    <span class="data-value" id="odom-z">0.000 m</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Linear Velocity:</span>
                    <span class="data-value" id="odom-vel-lin">0.000 m/s</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Angular Velocity:</span>
                    <span class="data-value" id="odom-vel-ang">0.000 rad/s</span>
                </div>
            </div>
            
            <!-- Command Velocity Card -->
            <div class="card">
                <div class="card-header">
                    <span class="icon">🎮</span>
                    <span>Command Velocity</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Linear:</span>
                    <span class="data-value" id="cmd-linear">0.000 m/s</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Angular:</span>
                    <span class="data-value" id="cmd-angular">0.000 rad/s</span>
                </div>
            </div>
            
            <!-- LiDAR Card -->
            <div class="card">
                <div class="card-header">
                    <span class="icon">📡</span>
                    <span>LiDAR / Laser Scan</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Average Range:</span>
                    <span class="data-value" id="scan-avg">0.000 m</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Minimum Range:</span>
                    <span class="data-value" id="scan-min">0.000 m</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Maximum Range:</span>
                    <span class="data-value" id="scan-max">0.000 m</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Valid Readings:</span>
                    <span class="data-value" id="scan-readings">0</span>
                </div>
                <div class="data-row">
                    <span class="data-label">Obstacles (<2m):</span>
                    <span class="data-value" id="scan-obstacles">0</span>
                </div>
            </div>
        </div>
        
        <!-- Charts Section -->
        <div class="grid">
            <div class="card full-width">
                <div class="card-header">
                    <span class="icon">📈</span>
                    <span>Orientation (Roll, Pitch, Yaw)</span>
                </div>
                <div class="chart-container">
                    <canvas id="orientationChart"></canvas>
                </div>
            </div>
            
            <div class="card full-width">
                <div class="card-header">
                    <span class="icon">📉</span>
                    <span>Acceleration</span>
                </div>
                <div class="chart-container">
                    <canvas id="accelerationChart"></canvas>
                </div>
            </div>
            
            <div class="card full-width">
                <div class="card-header">
                    <span class="icon">🚀</span>
                    <span>Velocity</span>
                </div>
                <div class="chart-container">
                    <canvas id="velocityChart"></canvas>
                </div>
            </div>
            
            <div class="card full-width">
                <div class="card-header">
                    <span class="icon">📡</span>
                    <span>LiDAR Range Data</span>
                </div>
                <div class="chart-container">
                    <canvas id="lidarChart"></canvas>
                </div>
            </div>
        </div>
        
        <!-- Alerts Section -->
        <div class="card full-width">
            <div class="card-header">
                <span class="icon">⚠️</span>
                <span>Alerts & Thresholds</span>
            </div>
            <div class="alert-config">
                <div>
                    <label style="color: #a0d8ff;">Max Velocity (m/s):</label>
                    <input type="number" id="threshold-velocity" step="0.1" value="2.0">
                </div>
                <div>
                    <label style="color: #a0d8ff;">Max Acceleration (m/s²):</label>
                    <input type="number" id="threshold-acceleration" step="0.5" value="15.0">
                </div>
                <div>
                    <label style="color: #a0d8ff;">Max Angular Velocity (rad/s):</label>
                    <input type="number" id="threshold-angular" step="0.1" value="3.0">
                </div>
                <div style="display: flex; align-items: flex-end;">
                    <button class="btn btn-primary" onclick="updateThresholds()" style="width: 100%;">
                        Update Thresholds
                    </button>
                </div>
            </div>
            <div class="alerts-container" id="alerts-container">
                <div style="text-align: center; padding: 20px; opacity: 0.5;">
                    No alerts yet
                </div>
            </div>
        </div>
        
        <!-- Joint States Card -->
        <div class="card full-width">
            <div class="card-header">
                <span class="icon">⚙️</span>
                <span>Joint States</span>
            </div>
            <table class="joints-table" id="joints-table">
                <tbody></tbody>
            </table>
        </div>
        
        <!-- Diagnostics Card -->
        <div class="card full-width">
            <div class="card-header">
                <span class="icon">🔧</span>
                <span>Diagnostics</span>
            </div>
            <div class="diagnostics-list" id="diagnostics-list">
                <div class="diagnostic-item">Waiting for diagnostics...</div>
            </div>
        </div>
        
        <div class="last-update">
            Last Update: <span id="last-update">Never</span>
        </div>
    </div>
    
    <script>
        // Initialize charts
        const orientationChart = new Chart(document.getElementById('orientationChart'), {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'Roll',
                        data: [],
                        borderColor: '#ff6384',
                        backgroundColor: 'rgba(255, 99, 132, 0.1)',
                        tension: 0.4
                    },
                    {
                        label: 'Pitch',
                        data: [],
                        borderColor: '#36a2eb',
                        backgroundColor: 'rgba(54, 162, 235, 0.1)',
                        tension: 0.4
                    },
                    {
                        label: 'Yaw',
                        data: [],
                        borderColor: '#ffce56',
                        backgroundColor: 'rgba(255, 206, 86, 0.1)',
                        tension: 0.4
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: {
                        labels: { color: '#fff' }
                    }
                },
                scales: {
                    x: { 
                        ticks: { color: '#fff' },
                        grid: { color: 'rgba(255,255,255,0.1)' }
                    },
                    y: { 
                        ticks: { color: '#fff' },
                        grid: { color: 'rgba(255,255,255,0.1)' }
                    }
                }
            }
        });
        
        const accelerationChart = new Chart(document.getElementById('accelerationChart'), {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'Acc X',
                        data: [],
                        borderColor: '#ff6384',
                        backgroundColor: 'rgba(255, 99, 132, 0.1)',
                        tension: 0.4
                    },
                    {
                        label: 'Acc Y',
                        data: [],
                        borderColor: '#36a2eb',
                        backgroundColor: 'rgba(54, 162, 235, 0.1)',
                        tension: 0.4
                    },
                    {
                        label: 'Acc Z',
                        data: [],
                        borderColor: '#ffce56',
                        backgroundColor: 'rgba(255, 206, 86, 0.1)',
                        tension: 0.4
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: {
                        labels: { color: '#fff' }
                    }
                },
                scales: {
                    x: { 
                        ticks: { color: '#fff' },
                        grid: { color: 'rgba(255,255,255,0.1)' }
                    },
                    y: { 
                        ticks: { color: '#fff' },
                        grid: { color: 'rgba(255,255,255,0.1)' }
                    }
                }
            }
        });
        
        const velocityChart = new Chart(document.getElementById('velocityChart'), {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Linear Velocity',
                    data: [],
                    borderColor: '#4bc0c0',
                    backgroundColor: 'rgba(75, 192, 192, 0.1)',
                    tension: 0.4
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: {
                        labels: { color: '#fff' }
                    }
                },
                scales: {
                    x: { 
                        ticks: { color: '#fff' },
                        grid: { color: 'rgba(255,255,255,0.1)' }
                    },
                    y: { 
                        ticks: { color: '#fff' },
                        grid: { color: 'rgba(255,255,255,0.1)' }
                    }
                }
            }
        });
        
        const lidarChart = new Chart(document.getElementById('lidarChart'), {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'Average Range',
                        data: [],
                        borderColor: '#9966ff',
                        backgroundColor: 'rgba(153, 102, 255, 0.1)',
                        tension: 0.4
                    },
                    {
                        label: 'Minimum Range',
                        data: [],
                        borderColor: '#ff6384',
                        backgroundColor: 'rgba(255, 99, 132, 0.1)',
                        tension: 0.4
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: {
                        labels: { color: '#fff' }
                    }
                },
                scales: {
                    x: { 
                        ticks: { color: '#fff' },
                        grid: { color: 'rgba(255,255,255,0.1)' }
                    },
                    y: { 
                        ticks: { color: '#fff' },
                        grid: { color: 'rgba(255,255,255,0.1)' }
                    }
                }
            }
        });
        
        function startLogging() {
            fetch('/api/logging/start', { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    document.getElementById('startLoggingBtn').disabled = true;
                    document.getElementById('stopLoggingBtn').disabled = false;
                    document.getElementById('statusBadge').className = 'status-badge status-recording';
                    document.getElementById('statusBadge').textContent = '🔴 RECORDING';
                });
        }
        
        function stopLogging() {
            fetch('/api/logging/stop', { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    document.getElementById('startLoggingBtn').disabled = false;
                    document.getElementById('stopLoggingBtn').disabled = true;
                    document.getElementById('statusBadge').className = 'status-badge status-idle';
                    document.getElementById('statusBadge').textContent = '⚪ IDLE';
                    alert(`Logging stopped. ${data.records} records captured.`);
                });
        }
        
        function exportCSV() {
            window.location.href = '/api/export/csv';
        }
        
        function clearAlerts() {
            fetch('/api/alerts/clear', { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    updateAlerts();
                });
        }
        
        function updateThresholds() {
            const velocity = document.getElementById('threshold-velocity').value;
            const acceleration = document.getElementById('threshold-acceleration').value;
            const angular = document.getElementById('threshold-angular').value;
            
            fetch('/api/alerts/configure', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    velocity: parseFloat(velocity),
                    acceleration: parseFloat(acceleration),
                    angular_velocity: parseFloat(angular)
                })
            })
            .then(response => response.json())
            .then(data => {
                alert('Thresholds updated successfully!');
            });
        }
        
        function updateDashboard() {
            fetch('/api/data')
                .then(response => response.json())
                .then(data => {
                    // Update IMU
                    document.getElementById('imu-roll').textContent = data.imu.roll.toFixed(3) + '°';
                    document.getElementById('imu-pitch').textContent = data.imu.pitch.toFixed(3) + '°';
                    document.getElementById('imu-yaw').textContent = data.imu.yaw.toFixed(3) + '°';
                    document.getElementById('imu-ang-x').textContent = data.imu.angular_vel.x.toFixed(3) + ' rad/s';
                    document.getElementById('imu-ang-y').textContent = data.imu.angular_vel.y.toFixed(3) + ' rad/s';
                    document.getElementById('imu-ang-z').textContent = data.imu.angular_vel.z.toFixed(3) + ' rad/s';
                    document.getElementById('imu-acc-x').textContent = data.imu.linear_acc.x.toFixed(3) + ' m/s²';
                    document.getElementById('imu-acc-y').textContent = data.imu.linear_acc.y.toFixed(3) + ' m/s²';
                    document.getElementById('imu-acc-z').textContent = data.imu.linear_acc.z.toFixed(3) + ' m/s²';
                    
                    // Update GPS
                    document.getElementById('gps-lat').textContent = data.gps.lat.toFixed(6) + '°';
                    document.getElementById('gps-lon').textContent = data.gps.lon.toFixed(6) + '°';
                    document.getElementById('gps-alt').textContent = data.gps.alt.toFixed(2) + ' m';
                    
                    // Update Odometry
                    document.getElementById('odom-x').textContent = data.odom.position.x.toFixed(3) + ' m';
                    document.getElementById('odom-y').textContent = data.odom.position.y.toFixed(3) + ' m';
                    document.getElementById('odom-z').textContent = data.odom.position.z.toFixed(3) + ' m';
                    document.getElementById('odom-vel-lin').textContent = data.odom.velocity.linear.toFixed(3) + ' m/s';
                    document.getElementById('odom-vel-ang').textContent = data.odom.velocity.angular.toFixed(3) + ' rad/s';
                    
                    // Update Command Velocity
                    document.getElementById('cmd-linear').textContent = data.cmd_vel.linear.toFixed(3) + ' m/s';
                    document.getElementById('cmd-angular').textContent = data.cmd_vel.angular.toFixed(3) + ' rad/s';
                    
                    // Update LiDAR
                    document.getElementById('scan-avg').textContent = data.scan.avg_range.toFixed(3) + ' m';
                    document.getElementById('scan-min').textContent = data.scan.min_range.toFixed(3) + ' m';
                    document.getElementById('scan-max').textContent = data.scan.max_range.toFixed(3) + ' m';
                    document.getElementById('scan-readings').textContent = data.scan.num_readings;
                    document.getElementById('scan-obstacles').textContent = data.scan.obstacles_detected;
                    
                    // Update Joints
                    const jointsTable = document.getElementById('joints-table').querySelector('tbody');
                    jointsTable.innerHTML = '';
                    data.joints.forEach(joint => {
                        const row = jointsTable.insertRow();
                        row.innerHTML = `
                            <td style="color: #a0d8ff;">${joint.name}</td>
                            <td>Pos: <span style="color: #fff;">${joint.position.toFixed(3)}</span></td>
                            <td>Vel: <span style="color: #fff;">${joint.velocity.toFixed(3)}</span></td>
                        `;
                    });
                    
                    // Update Diagnostics
                    const diagList = document.getElementById('diagnostics-list');
                    diagList.innerHTML = '';
                    if (data.diagnostics.length > 0) {
                        data.diagnostics.forEach(diag => {
                            const item = document.createElement('div');
                            item.className = 'diagnostic-item';
                            item.textContent = '✓ ' + diag;
                            diagList.appendChild(item);
                        });
                    } else {
                        diagList.innerHTML = '<div class="diagnostic-item">No diagnostics available</div>';
                    }
                    
                    // Update timestamp
                    document.getElementById('last-update').textContent = new Date(data.last_update).toLocaleString();
                })
                .catch(error => console.error('Error fetching data:', error));
        }
        
        function updateCharts() {
            fetch('/api/history')
                .then(response => response.json())
                .then(data => {
                    const labels = data.timestamps.map((t, i) => i);
                    
                    // Update orientation chart
                    orientationChart.data.labels = labels;
                    orientationChart.data.datasets[0].data = data.imu_roll;
                    orientationChart.data.datasets[1].data = data.imu_pitch;
                    orientationChart.data.datasets[2].data = data.imu_yaw;
                    orientationChart.update('none');
                    
                    // Update acceleration chart
                    accelerationChart.data.labels = labels;
                    accelerationChart.data.datasets[0].data = data.acceleration_x;
                    accelerationChart.data.datasets[1].data = data.acceleration_y;
                    accelerationChart.data.datasets[2].data = data.acceleration_z;
                    accelerationChart.update('none');
                    
                    // Update velocity chart
                    velocityChart.data.labels = labels;
                    velocityChart.data.datasets[0].data = data.velocity;
                    velocityChart.update('none');
                    
                    // Update LiDAR chart
                    lidarChart.data.labels = labels;
                    lidarChart.data.datasets[0].data = data.scan_avg_range;
                    lidarChart.data.datasets[1].data = data.scan_min_range;
                    lidarChart.update('none');
                })
                .catch(error => console.error('Error fetching history:', error));
        }
        
        function updateAlerts() {
            fetch('/api/alerts')
                .then(response => response.json())
                .then(data => {
                    const container = document.getElementById('alerts-container');
                    container.innerHTML = '';
                    
                    if (data.alerts.length > 0) {
                        data.alerts.slice().reverse().forEach(alert => {
                            const item = document.createElement('div');
                            item.className = 'alert-item';
                            item.textContent = '⚠️ ' + alert;
                            container.appendChild(item);
                        });
                    } else {
                        container.innerHTML = '<div style="text-align: center; padding: 20px; opacity: 0.5;">No alerts yet</div>';
                    }
                });
        }
        
        function updateLoggingStatus() {
            fetch('/api/logging/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('recordCount').textContent = `Records: ${data.records}`;
                });
        }
        
        function loadThresholds() {
            fetch('/api/alerts/thresholds')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('threshold-velocity').value = data.velocity;
                    document.getElementById('threshold-acceleration').value = data.acceleration;
                    document.getElementById('threshold-angular').value = data.angular_velocity;
                });
        }
        
        // Update intervals
        setInterval(updateDashboard, 100);
        setInterval(updateCharts, 500);
        setInterval(updateAlerts, 1000);
        setInterval(updateLoggingStatus, 500);
        
        // Initial load
        updateDashboard();
        updateCharts();
        updateAlerts();
        loadThresholds();
    </script>
</body>
</html>'''
    
    with open(os.path.join(template_dir, 'dashboard.html'), 'w') as f:
        f.write(html_content)

def main():
    global dashboard_node, anomaly_detector
    
    rclpy.init()
    dashboard_node = HuskyDashboard()
    
    # Try to load existing anomaly detection model
    if ANOMALY_DETECTION_AVAILABLE and anomaly_detector:
        try:
            anomaly_detector.load_model()
            print("✅ Anomaly detection model loaded successfully")
        except FileNotFoundError:
            print("ℹ️  No pre-trained anomaly model found (train one with train_anomaly_model.py)")
        except Exception as e:
            print(f"⚠️  Error loading anomaly model: {str(e)}")
    else:
        print("ℹ️  Anomaly detection module not available")
    
    # Create HTML template
    create_html_template()
    
    # Start ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    print("\n" + "="*60)
    print("🚀 Enhanced Husky Dashboard Server Starting...")
    print("="*60)
    print("\n📊 Open your browser and navigate to:")
    print("\n    http://localhost:5000")
    print("\n✨ Features:")
    print("   • Real-time sensor data visualization")
    print("   • Interactive charts (Orientation, Acceleration, Velocity)")
    print("   • Data logging with CSV export")
    print("   • Configurable alert thresholds")
    print("   • Alert monitoring")
    print("\n💪 Production Server:")
    print("   • Using Waitress WSGI server (production-ready)")
    print("   • Optimized for performance and stability")
    print("\n" + "="*60)
    print("Press Ctrl+C to stop the server")
    print("="*60 + "\n")
    
    try:
        serve(app, host='0.0.0.0', port=5000, threads=4)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        dashboard_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

