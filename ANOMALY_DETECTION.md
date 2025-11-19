# 🤖 Husky Robot Anomaly Detection System

This system uses machine learning to detect unusual/abnormal sensor patterns in your Husky robot data.

## 📋 Overview

The anomaly detection system learns what "normal" behavior looks like from your recorded data, then flags unusual patterns in real-time. This can help identify:

- ⚠️ Sensor malfunctions
- 🔧 Mechanical issues
- 🚨 Unusual robot behavior
- 📊 Data quality problems

## 🚀 Quick Start Guide

### Step 1: Install Dependencies

```bash
cd /home/team5/husky-dashboard
pip3 install -r requirements.txt
```

### Step 2: Collect Training Data

1. Start the dashboard:
   ```bash
   python3 husky_web_dashboard.py
   ```

2. Open browser: `http://localhost:5000`

3. Click "▶️ Start Logging" and operate your robot normally for a few minutes

4. Click "⏹️ Stop Logging"

5. Click "📥 Export CSV" to download your training data

### Step 3: Train the Model

```bash
python3 train_anomaly_model.py /tmp/husky_data_YYYYMMDD_HHMMSS.csv
```

**Optional**: Adjust sensitivity with contamination parameter (default: 0.1)
```bash
# More sensitive (strict): 0.05 = 5% outliers expected
python3 train_anomaly_model.py your_data.csv 0.05

# Less sensitive (lenient): 0.2 = 20% outliers expected
python3 train_anomaly_model.py your_data.csv 0.2
```

### Step 4: Run Real-Time Detection

The model automatically loads when you start the dashboard:

```bash
python3 husky_web_dashboard.py
```

Anomalies will be detected and displayed in real-time! 🎉

## 📊 What Gets Analyzed

The system analyzes these sensor features:

### IMU Data
- Roll, Pitch, Yaw (orientation)
- Angular velocities (X, Y, Z)
- Linear accelerations (X, Y, Z)
- Total acceleration magnitude
- Angular velocity magnitude

### Motion Data
- Linear velocity
- Angular velocity
- Command velocities

## 🎯 How It Works

### Algorithm: Isolation Forest

The system uses **Isolation Forest**, a powerful anomaly detection algorithm that:

1. **Learns Normal Patterns**: During training, it builds a model of what normal robot behavior looks like
2. **Detects Outliers**: New data points far from normal patterns get flagged as anomalies
3. **Anomaly Score**: Each data point gets a score (lower = more anomalous)

### Feature Engineering

The system extracts and combines multiple sensor readings to create a comprehensive picture of robot state.

## 🔧 Advanced Usage

### Command-Line Training

```bash
# Basic training
python3 anomaly_detector.py train your_data.csv

# Test on new data
python3 anomaly_detector.py test test_data.csv
```

### API Endpoints

Once the dashboard is running, you can use these endpoints:

#### Train Model
```bash
curl -X POST http://localhost:5000/api/anomaly/train \
  -H "Content-Type: application/json" \
  -d '{"csv_file": "/tmp/husky_data.csv", "contamination": 0.1}'
```

#### Load Model
```bash
curl -X POST http://localhost:5000/api/anomaly/load
```

#### Get Anomaly Status
```bash
curl http://localhost:5000/api/anomaly/status
```

#### Get Anomaly Alerts
```bash
curl http://localhost:5000/api/anomaly/alerts
```

## 📈 Interpreting Results

### Anomaly Score
- **Score > 0**: Normal behavior
- **Score ≈ 0**: Borderline
- **Score < -0.1**: Likely anomaly (default threshold)
- **Score < -0.5**: Definite anomaly

### When to Investigate
Investigate anomalies that:
- Occur repeatedly
- Happen during specific maneuvers
- Correlate with physical robot issues
- Show extreme score values (< -0.5)

## 🎓 Best Practices

### Training Data Quality

**DO:**
- ✅ Collect data during normal operation
- ✅ Include various maneuvers (straight, turns, stops)
- ✅ Use at least 5-10 minutes of data
- ✅ Ensure sensors are working properly during collection

**DON'T:**
- ❌ Include data from malfunctioning sensors
- ❌ Train on only stationary data
- ❌ Use data with known issues

### Contamination Parameter

Choose based on your data quality:

| Contamination | Use When | Detection |
|--------------|----------|-----------|
| 0.05 (5%) | Clean data, strict detection | More false alarms |
| 0.1 (10%) | **Recommended default** | Balanced |
| 0.15 (15%) | Noisy data | May miss some anomalies |
| 0.2 (20%) | Very noisy data, lenient | Fewer false alarms |

## 🔍 Troubleshooting

### "Model not trained" error
- **Solution**: Run `train_anomaly_model.py` with your CSV data first

### Too many false positives
- **Solution**: Retrain with higher contamination (e.g., 0.15 or 0.2)

### Missing anomalies
- **Solution**: Retrain with lower contamination (e.g., 0.05)
- **Check**: Is your training data truly "normal"?

### Import errors
- **Solution**: Install dependencies: `pip3 install -r requirements.txt`

## 📁 Files

```
husky-dashboard/
├── anomaly_detector.py          # Core ML model
├── train_anomaly_model.py       # Easy training script
├── husky_web_dashboard.py       # Main dashboard (with anomaly detection)
├── models/                      # Saved models directory
│   └── husky_anomaly_model.pkl  # Trained model (after training)
└── requirements.txt             # Dependencies
```

## 🎯 Example Workflow

```bash
# 1. Collect normal operation data
python3 husky_web_dashboard.py
# (Use dashboard to log and export data)

# 2. Train the model
python3 train_anomaly_model.py /tmp/husky_data_20251116_190255.csv

# 3. Run real-time detection
python3 husky_web_dashboard.py

# 4. Play bag file and watch for anomalies
ros2 bag play /home/team5/Downloads/Husky
```

## 🔬 Technical Details

- **Algorithm**: Isolation Forest
- **Preprocessing**: StandardScaler + PCA
- **Features**: 14 sensor features + 2 derived features
- **Model Storage**: Joblib (PKL format)
- **Real-time Processing**: ~1ms per prediction

## 📚 References

- Isolation Forest: [Liu et al. 2008](https://cs.nju.edu.cn/zhouzh/zhouzh.files/publication/icdm08b.pdf)
- Scikit-learn: https://scikit-learn.org/stable/modules/outlier_detection.html

---

💡 **Need Help?** Check the main README.md or open an issue on GitHub!


