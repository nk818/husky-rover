

---

# Husky Robot Sensor Dashboard

A real-time monitoring, data validation, and security-testing dashboard for the Clearpath Husky UGV running ROS2 Foxy.
Supports IMU, GPS, Odometry, LiDAR, Joint States, Cmd Vel, and Diagnostics.

---

## Features

### Real-Time Monitoring

* IMU (orientation, angular velocity, linear acceleration)
* GPS (lat, lon, altitude)
* Odometry (position & velocity)
* LiDAR (360° scan + obstacle detection)
* Joint states and command velocities
* Diagnostic messages

### Data Visualization

* Orientation, acceleration, and velocity charts
* Real-time 360° LiDAR top-down display
* Stores last 100 data points

### Logging & Export

* Start/stop data logging
* Export CSV with timestamps and all sensor fields
* **LiDAR/Laser scan data recording** (avg/min/max range, obstacles)
* See [LIDAR_RECORDING.md](LIDAR_RECORDING.md) for LiDAR details

### 🆕 **Machine Learning Anomaly Detection**

* **Train on normal robot behavior** from logged data
* **Real-time anomaly detection** using Isolation Forest
* **Automatic alerts** for unusual sensor patterns
* **Identify**: sensor malfunctions, mechanical issues, abnormal behavior
* See [ANOMALY_DETECTION.md](ANOMALY_DETECTION.md) for complete guide

### Security Testing

* IMU spike injection
* GPS drift injection
* Velocity spike
* Sensor freeze (replay)
* Random noise
* Automated anomaly detection
* Manual validation override

---

## Prerequisites

### Software

* Ubuntu 20.04
* ROS2 Foxy
* Python 3.8+
* Gazebo 11

### ROS2 Packages

```
sudo apt install ros-foxy-husky-simulator
sudo apt install ros-foxy-husky-gazebo
sudo apt install ros-foxy-rosbag2
sudo apt install ros-foxy-rosbag2-bag-v2-plugins
```

### Python Dependencies

```
pip3 install -r requirements.txt
```

Or manually:
```
pip3 install flask flask-cors numpy waitress scikit-learn pandas joblib
```

### Optional (ROS1 bag conversion):

```
pip3 install rosbags
```

---

## Installation

Clone:

```
git clone https://github.com/yourusername/husky-dashboard.git
cd husky-dashboard
```

Make executable:

```
chmod +x husky_web_dashboard.py
```

Verify ROS2:

```
source /opt/ros/foxy/setup.bash
ros2 pkg list | grep husky
```

---

## Usage

### Quick Start: Anomaly Detection

See [ANOMALY_DETECTION.md](ANOMALY_DETECTION.md) for complete ML setup guide.

**TL;DR:**
1. Collect data → Export CSV from dashboard
2. Train model: `python3 train_anomaly_model.py your_data.csv`
3. Run dashboard → Model auto-loads and detects anomalies!

### Terminal 1: Launch Husky in Gazebo

```
source /opt/ros/foxy/setup.bash
ros2 launch husky_gazebo husky_playpen.launch.py
```

### Terminal 2: Start Dashboard

```
cd ~/husky-dashboard
source /opt/ros/foxy/setup.bash
python3 husky_web_dashboard.py
```

Open in browser:

```
http://localhost:5000
```

### Terminal 3: Play ROS2 Bag

```
cd <bag_directory>
source /opt/ros/foxy/setup.bash
ros2 bag play husky
```

### Convert ROS1 Bags

```
rosbags-convert filename.bag
```

---

## Dashboard Overview

* Control bar: logging, CSV export, alerts, validation toggle
* Sensor panels: IMU, GPS, Odom, LiDAR, Joint States, Cmd Vel
* Charts: orientation, acceleration, velocity
* LiDAR view: 360° display with obstacle detection
* Security panel: start/stop injection attacks

---

## Configuration

### Adjustable alert thresholds (UI):

* Max velocity
* Max acceleration
* Max angular velocity

### Detection thresholds (code):

```
imu_jump: 45.0
acceleration_spike: 20.0
gps_jump: 0.01
velocity_jump: 2.0
lidar_avg_jump: 5.0
```

---

## CSV Export Fields

* Timestamp
* IMU: roll, pitch, yaw, ang_vel, lin_acc
* GPS: lat, lon, alt
* Odom: pos, vel
* Cmd vel
* LiDAR stats
* Injection status and type

---

## Troubleshooting

No data:

* Ensure Gazebo is running
* Run `ros2 topic list`
* Check `ros2 topic hz /imu/data`

LiDAR missing:

* Verify `/scan` exists
* Confirm Husky model includes LiDAR

Bag won't play:

* Convert ROS1 bags
* Check with `ros2 bag info`

Empty CSV:

* Start logging before playing bags

---

## Subscribed Topics

* `/imu/data`
* `/gps/data`
* `/scan`
* `/odom`
* `/joint_states`
* `/cmd_vel`
* `/diagnostics`

---

## Contributing

1. Fork repo
2. Create branch
3. Commit changes
4. Push and open PR

---

## License

MIT License (see LICENSE file)

---

## Support

Email: **[tmusangu@ttu.edu](mailto:tmusangu@ttu.edu)**

---

If you'd like, I can also generate:

* a minimal README
* a super-short README
* or a version formatted specifically for GitHub Markdown but still nano-friendly
