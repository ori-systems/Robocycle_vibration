# IMU Rosbag Data Visualization & Playback

📌 **A ROS-based Python script for playing rosbag files, extracting IMU data, and visualizing vibration magnitudes in real-time.**

## 🚀 Overview
This repository contains a ROS package that automates the playback of rosbag files, extracts IMU (Inertial Measurement Unit) data, and visualizes acceleration magnitudes using Matplotlib. The script supports multiple bag file playing.

## 🔥 Features
- ✅ **Rosbag Playback** –  play multiple rosbag files sequentially from script autorun.
- ✅ **IMU Data Extraction** – Reads `/imu/data` topic and computes acceleration magnitude.
- ✅ **Real-time Plotting** – Uses Matplotlib to display IMU vibration patterns dynamically.
- ✅ **Sliding Window Processing** – Maintains a fixed-size window (default: 500 samples).
---

## 🛠 Installation & Setup

### **1. Prerequisites**
Ensure you have the following installed:
- **ROS (Robot Operating System)** – Noetic.
- **Python 3** with required dependencies:
  ```bash
  pip install matplotlib rospy rosbag
  ```

### **2. Clone the Repository**
```bash
cd ~/catkin_ws/src
git clone https://github.com/your-username/imuvib_data.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### **3. Make Script Executable**
```bash
chmod +x ~/catkin_ws/src/imuvib_data/scripts/rosbag_playing.py
chmod +x ~/catkin_ws/src/imuvib_data/scripts/topic_subscribing.py
```

---

## ▶️ Usage
### **Run the Node**
Start ROS:
```bash
roscore
```
Then, execute the node:
```bash
roslaunch vib_data rosbag_play.launch 
```



## 🏗 How It Works
1. **Select Rosbag Files** – Opens a file dialog for choosing `.bag` files.
2. **Play Rosbags** – Sequentially plays selected bags.
3. **Extract IMU Data** – Reads acceleration from `/imu/data` and computes magnitude.
4. **Plot Data** – Displays a real-time graph of IMU vibration patterns.


