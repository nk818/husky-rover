#!/usr/bin/env python3
"""
Quick script to check if your CSV has real LiDAR data
"""
import pandas as pd
import sys

if len(sys.argv) < 2:
    print("Usage: python3 check_lidar_data.py <csv_file>")
    print("\nExample:")
    print("  python3 check_lidar_data.py ~/Downloads/husky_data_*.csv")
    sys.exit(1)

csv_file = sys.argv[1]

print(f"📊 Analyzing: {csv_file}\n")

df = pd.read_csv(csv_file)

print("=" * 60)
print("LIDAR DATA CHECK")
print("=" * 60)

# Check if LiDAR columns exist
lidar_cols = ['scan_avg_range', 'scan_min_range', 'scan_max_range', 
              'scan_num_readings', 'scan_obstacles_detected']

missing = [col for col in lidar_cols if col not in df.columns]

if missing:
    print(f"❌ Missing LiDAR columns: {missing}")
    print("\nYour CSV doesn't have LiDAR data structure.")
else:
    print("✅ All LiDAR columns present\n")
    
    # Check if data is non-zero
    print("LiDAR Statistics:")
    print("-" * 60)
    
    for col in lidar_cols:
        non_zero = (df[col] != 0).sum()
        total = len(df)
        percentage = (non_zero / total) * 100 if total > 0 else 0
        
        print(f"{col:30s}: {non_zero:6d}/{total} non-zero ({percentage:.1f}%)")
        
        if non_zero > 0:
            print(f"  → Min: {df[col].min():.3f}, Max: {df[col].max():.3f}, Avg: {df[col].mean():.3f}")
    
    print("\n" + "=" * 60)
    
    # Overall verdict
    if (df['scan_avg_range'] != 0).sum() > 0:
        print("✅ SUCCESS! You have real LiDAR data!")
        print(f"\n📊 Sample LiDAR readings:")
        print(df[lidar_cols].head(10))
    else:
        print("❌ WARNING: All LiDAR values are zero")
        print("\nThis means:")
        print("  1. The /scan topic wasn't publishing")
        print("  2. The bag file has no LiDAR data")
        print("  3. Try recording from live Gazebo simulation instead")

print("\n" + "=" * 60)


