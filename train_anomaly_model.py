#!/usr/bin/env python3
"""
Easy script to train anomaly detection model for Husky robot data
"""

import sys
import os
from anomaly_detector import HuskyAnomalyDetector

def main():
    print("="*60)
    print("🤖 Husky Robot Anomaly Detection Training")
    print("="*60)
    
    # Check if CSV file is provided
    if len(sys.argv) < 2:
        print("\n❌ Please provide a CSV file path")
        print("\nUsage:")
        print("  python train_anomaly_model.py <csv_file> [contamination]")
        print("\nExample:")
        print("  python train_anomaly_model.py /tmp/husky_data_20251116_190255.csv 0.1")
        print("\ncontamination: Expected proportion of outliers (default: 0.1 = 10%)")
        print("               Lower = stricter anomaly detection")
        print("               Higher = more lenient anomaly detection")
        return
    
    csv_file = sys.argv[1]
    contamination = float(sys.argv[2]) if len(sys.argv) > 2 else 0.1
    
    # Check if file exists
    if not os.path.exists(csv_file):
        print(f"\n❌ File not found: {csv_file}")
        return
    
    print(f"\n📁 Training data: {csv_file}")
    print(f"🎯 Contamination: {contamination} ({contamination*100:.1f}% expected anomalies)")
    print()
    
    # Create detector and train
    detector = HuskyAnomalyDetector()
    
    try:
        scores = detector.train(csv_file, contamination=contamination)
        detector.save_model()
        
        print("\n" + "="*60)
        print("✅ SUCCESS! Model trained and saved")
        print("="*60)
        print("\n📝 Next steps:")
        print("  1. Start your dashboard: python husky_web_dashboard.py")
        print("  2. The model will automatically load and detect anomalies")
        print("  3. Watch the dashboard for real-time anomaly alerts!")
        print()
        
    except Exception as e:
        print(f"\n❌ Training failed: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()


