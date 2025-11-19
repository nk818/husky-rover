#!/usr/bin/env python3
import sys
print("Python version:", sys.version, flush=True)
print("Starting import test...", flush=True)

try:
    from anomaly_detector import HuskyAnomalyDetector
    print("✅ Import successful", flush=True)
    
    import pandas as pd
    print("✅ Pandas imported", flush=True)
    
    detector = HuskyAnomalyDetector()
    print("✅ Detector created", flush=True)
    
    print("🎓 Loading data...", flush=True)
    csv_file = '/home/team5/Downloads/husky_data_20251116_190255.csv'
    
    print(f"Training on: {csv_file}", flush=True)
    detector.train(csv_file, contamination=0.1)
    
    print("💾 Saving model...", flush=True)
    detector.save_model()
    
    print("✅ SUCCESS! Training complete!", flush=True)
    
except Exception as e:
    print(f"❌ ERROR: {e}", flush=True)
    import traceback
    traceback.print_exc()
    sys.exit(1)


