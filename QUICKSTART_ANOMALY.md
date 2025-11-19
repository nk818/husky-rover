# 🚀 Quick Start: Anomaly Detection for Your Husky Data

## ✅ What You Have

You have existing data in `/home/team5/Downloads/husky_data_20251116_190255.csv` - **Perfect for training!**

## 📝 Step-by-Step Instructions

### 1️⃣ Train Your Model (5 minutes)

```bash
cd /home/team5/husky-dashboard
python3 train_anomaly_model.py /home/team5/Downloads/husky_data_20251116_190255.csv
```

**What this does:**
- Learns what "normal" sensor behavior looks like from your data
- Saves a trained model to `models/husky_anomaly_model.pkl`
- Shows statistics about your training data

**Expected Output:**
```
🤖 Husky Robot Anomaly Detection Training
============================================================
📁 Training data: /home/team5/Downloads/husky_data_20251116_190255.csv
🎯 Contamination: 0.1 (10.0% expected anomalies)

📊 Loading training data...
✓ Loaded XXXX records
🔧 Extracting features...
✓ Extracted 16 features from data
📏 Scaling features...
🎯 Applying PCA...
🌲 Training Isolation Forest...
✅ Training complete!
```

### 2️⃣ Play Your Bag File with Real-Time Detection

**Terminal 1: Start Dashboard**
```bash
cd /home/team5/husky-dashboard
python3 husky_web_dashboard.py
```

You'll see:
```
✅ Anomaly detection model loaded successfully
```

**Terminal 2: Play Bag File**
```bash
ros2 bag play /home/team5/Downloads/Husky
```

### 3️⃣ Watch for Anomalies

Open browser: **http://localhost:5000**

**What you'll see:**
- ✅ Green status = Normal behavior
- 🚨 Red alerts = Anomalies detected!
- 📊 Real-time sensor data
- ⚠️ Anomaly alerts list

## 🔍 What Gets Detected

The system will flag unusual patterns in:
- **IMU**: Sudden orientation changes, unusual angular velocities
- **Acceleration**: Spikes or drops in linear acceleration
- **Velocity**: Unexpected speed changes
- **Motion**: Unusual combinations of linear/angular movement

## 🎯 Adjust Sensitivity

### Too Many False Alarms?
Retrain with higher contamination (more lenient):
```bash
python3 train_anomaly_model.py /home/team5/Downloads/husky_data_20251116_190255.csv 0.15
```

### Missing Anomalies?
Retrain with lower contamination (stricter):
```bash
python3 train_anomaly_model.py /home/team5/Downloads/husky_data_20251116_190255.csv 0.05
```

## 📊 Test on Other Bag Files

```bash
# Train on normal data
python3 train_anomaly_model.py /home/team5/Downloads/husky_data_20251116_190255.csv

# Test on different scenarios
ros2 bag play /home/team5/Downloads/themaze
ros2 bag play /home/team5/Downloads/vel
ros2 bag play /home/team5/Downloads/subset
```

## 🔧 Advanced: API Usage

While dashboard is running:

**Check anomaly status:**
```bash
curl http://localhost:5000/api/anomaly/status
```

**Get all anomaly alerts:**
```bash
curl http://localhost:5000/api/anomaly/alerts
```

## 💡 Tips

1. **Best Training Data**: Use data from normal operation, various maneuvers
2. **Data Amount**: 5-10 minutes of logged data is usually enough
3. **Contamination**: Start with 0.1 (10%), adjust based on results
4. **Real-time**: Model runs fast (~1ms per prediction), no lag

## 📚 More Info

- Full documentation: `ANOMALY_DETECTION.md`
- Technical details: `anomaly_detector.py`
- Dashboard code: `husky_web_dashboard.py`

---

## 🎓 Example Complete Workflow

```bash
# Step 1: Train
cd /home/team5/husky-dashboard
python3 train_anomaly_model.py /home/team5/Downloads/husky_data_20251116_190255.csv

# Step 2: Start dashboard (in Terminal 1)
python3 husky_web_dashboard.py

# Step 3: Play bag (in Terminal 2)
ros2 bag play /home/team5/Downloads/Husky

# Step 4: Watch browser at http://localhost:5000 for anomalies!
```

---

**Questions?** Check the main documentation or the code comments!


