#!/usr/bin/env python3
"""
Anomaly Detection System for Husky Robot Sensor Data
Uses multiple ML approaches to detect unusual patterns
"""

import numpy as np
import pandas as pd
from sklearn.ensemble import IsolationForest
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
import joblib
import json
from datetime import datetime
import os

class HuskyAnomalyDetector:
    def __init__(self, model_path='models/'):
        self.model_path = model_path
        os.makedirs(model_path, exist_ok=True)
        
        self.isolation_forest = None
        self.scaler = StandardScaler()
        self.pca = None
        self.feature_names = []
        self.is_trained = False
        
        # Thresholds for different anomaly scores
        self.anomaly_threshold = -0.1  # Isolation Forest threshold
        
    def extract_features(self, data_entry):
        """Extract relevant features from a data entry"""
        features = []
        feature_names = []
        
        # IMU features
        if 'imu_roll' in data_entry:
            features.extend([
                data_entry['imu_roll'],
                data_entry['imu_pitch'],
                data_entry['imu_yaw'],
                data_entry['imu_ang_vel_x'],
                data_entry['imu_ang_vel_y'],
                data_entry['imu_ang_vel_z'],
                data_entry['imu_lin_acc_x'],
                data_entry['imu_lin_acc_y'],
                data_entry['imu_lin_acc_z']
            ])
            feature_names.extend([
                'imu_roll', 'imu_pitch', 'imu_yaw',
                'ang_vel_x', 'ang_vel_y', 'ang_vel_z',
                'acc_x', 'acc_y', 'acc_z'
            ])
        
        # Odometry features
        if 'odom_vel_linear' in data_entry:
            features.extend([
                data_entry['odom_vel_linear'],
                data_entry['odom_vel_angular']
            ])
            feature_names.extend(['vel_linear', 'vel_angular'])
        
        # Command velocity features
        if 'cmd_vel_linear' in data_entry:
            features.extend([
                data_entry['cmd_vel_linear'],
                data_entry['cmd_vel_angular']
            ])
            feature_names.extend(['cmd_vel_linear', 'cmd_vel_angular'])
        
        # Derived features (useful for anomaly detection)
        if len(features) >= 9:
            # Total acceleration magnitude
            acc_magnitude = np.sqrt(
                data_entry['imu_lin_acc_x']**2 + 
                data_entry['imu_lin_acc_y']**2 + 
                data_entry['imu_lin_acc_z']**2
            )
            features.append(acc_magnitude)
            feature_names.append('acc_magnitude')
            
            # Angular velocity magnitude
            ang_vel_magnitude = np.sqrt(
                data_entry['imu_ang_vel_x']**2 + 
                data_entry['imu_ang_vel_y']**2 + 
                data_entry['imu_ang_vel_z']**2
            )
            features.append(ang_vel_magnitude)
            feature_names.append('ang_vel_magnitude')
        
        return np.array(features), feature_names
    
    def train(self, csv_file_path, contamination=0.1):
        """
        Train the anomaly detection model on normal data
        
        Args:
            csv_file_path: Path to CSV file with logged data
            contamination: Expected proportion of outliers (0.05-0.2)
        """
        print("📊 Loading training data...")
        df = pd.read_csv(csv_file_path)
        print(f"✓ Loaded {len(df)} records")
        
        # Extract features
        print("🔧 Extracting features...")
        features_list = []
        for _, row in df.iterrows():
            feats, feat_names = self.extract_features(row.to_dict())
            features_list.append(feats)
        
        X = np.array(features_list)
        self.feature_names = feat_names
        print(f"✓ Extracted {X.shape[1]} features from data")
        
        # Handle NaN and infinite values
        X = np.nan_to_num(X, nan=0.0, posinf=0.0, neginf=0.0)
        
        # Scale features
        print("📏 Scaling features...")
        X_scaled = self.scaler.fit_transform(X)
        
        # Apply PCA for dimensionality reduction (optional, good for visualization)
        print("🎯 Applying PCA...")
        self.pca = PCA(n_components=min(10, X.shape[1]))
        X_pca = self.pca.fit_transform(X_scaled)
        
        # Train Isolation Forest
        print("🌲 Training Isolation Forest...")
        self.isolation_forest = IsolationForest(
            contamination=contamination,
            random_state=42,
            n_estimators=100
        )
        self.isolation_forest.fit(X_pca)
        
        self.is_trained = True
        print("✅ Training complete!")
        
        # Show training statistics
        scores = self.isolation_forest.score_samples(X_pca)
        print(f"\n📈 Training Statistics:")
        print(f"   Mean anomaly score: {scores.mean():.3f}")
        print(f"   Std anomaly score: {scores.std():.3f}")
        print(f"   Min score: {scores.min():.3f}")
        print(f"   Max score: {scores.max():.3f}")
        
        return scores
    
    def predict(self, data_entry):
        """
        Predict if a data entry is anomalous
        
        Returns:
            is_anomaly: bool - True if anomalous
            score: float - Anomaly score (lower = more anomalous)
            features: dict - Feature values for debugging
        """
        if not self.is_trained:
            return False, 0.0, {}
        
        # Extract features
        features, _ = self.extract_features(data_entry)
        features = features.reshape(1, -1)
        
        # Handle NaN and infinite values
        features = np.nan_to_num(features, nan=0.0, posinf=0.0, neginf=0.0)
        
        # Scale and transform
        features_scaled = self.scaler.transform(features)
        features_pca = self.pca.transform(features_scaled)
        
        # Predict
        score = self.isolation_forest.score_samples(features_pca)[0]
        is_anomaly = score < self.anomaly_threshold
        
        # Create feature dict for debugging
        feature_dict = {name: float(val) for name, val in zip(self.feature_names, features[0])}
        
        return is_anomaly, float(score), feature_dict
    
    def save_model(self, name='husky_anomaly_model'):
        """Save the trained model"""
        if not self.is_trained:
            raise ValueError("Model must be trained before saving")
        
        model_data = {
            'isolation_forest': self.isolation_forest,
            'scaler': self.scaler,
            'pca': self.pca,
            'feature_names': self.feature_names,
            'anomaly_threshold': self.anomaly_threshold
        }
        
        filepath = os.path.join(self.model_path, f'{name}.pkl')
        joblib.dump(model_data, filepath)
        print(f"✅ Model saved to {filepath}")
    
    def load_model(self, name='husky_anomaly_model'):
        """Load a trained model"""
        filepath = os.path.join(self.model_path, f'{name}.pkl')
        
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Model not found: {filepath}")
        
        model_data = joblib.load(filepath)
        self.isolation_forest = model_data['isolation_forest']
        self.scaler = model_data['scaler']
        self.pca = model_data['pca']
        self.feature_names = model_data['feature_names']
        self.anomaly_threshold = model_data['anomaly_threshold']
        self.is_trained = True
        
        print(f"✅ Model loaded from {filepath}")
    
    def evaluate_dataset(self, csv_file_path):
        """Evaluate entire dataset and return anomaly statistics"""
        df = pd.read_csv(csv_file_path)
        
        anomalies = []
        scores = []
        
        for idx, row in df.iterrows():
            is_anomaly, score, features = self.predict(row.to_dict())
            anomalies.append(is_anomaly)
            scores.append(score)
        
        anomaly_indices = [i for i, is_anom in enumerate(anomalies) if is_anom]
        
        print(f"\n📊 Dataset Evaluation:")
        print(f"   Total records: {len(df)}")
        print(f"   Anomalies detected: {sum(anomalies)} ({100*sum(anomalies)/len(df):.1f}%)")
        print(f"   Mean score: {np.mean(scores):.3f}")
        
        return anomalies, scores, anomaly_indices


def main():
    """Example usage"""
    import sys
    
    if len(sys.argv) < 2:
        print("Usage:")
        print("  Train: python anomaly_detector.py train <csv_file>")
        print("  Test:  python anomaly_detector.py test <csv_file>")
        return
    
    command = sys.argv[1]
    detector = HuskyAnomalyDetector()
    
    if command == 'train':
        if len(sys.argv) < 3:
            print("Please provide CSV file path")
            return
        
        csv_file = sys.argv[2]
        print(f"🎓 Training on {csv_file}")
        detector.train(csv_file, contamination=0.1)
        detector.save_model()
        
    elif command == 'test':
        if len(sys.argv) < 3:
            print("Please provide CSV file path")
            return
        
        csv_file = sys.argv[2]
        print(f"🔍 Testing on {csv_file}")
        
        try:
            detector.load_model()
            anomalies, scores, anomaly_indices = detector.evaluate_dataset(csv_file)
            
            print(f"\n🚨 Anomalous records at indices: {anomaly_indices[:10]}...")
            
        except FileNotFoundError:
            print("❌ No trained model found. Please train first.")
    
    else:
        print(f"Unknown command: {command}")


if __name__ == '__main__':
    main()


