# IMU Rosbag Data Visualization & Playback

📌 **A ROS-based Python script for playing rosbag files, extracting IMU data, and visualizing vibration magnitudes in real-time.**

## 🚀 Overview
This repository contains a ROS node that automates the playback of rosbag files, extracts IMU (Inertial Measurement Unit) data, and visualizes acceleration magnitudes using Matplotlib. The script supports multiple bag file selection and runs in a multi-threaded environment to ensure smooth execution.

## 🔥 Features
- ✅ **Rosbag Playback** – Select and play multiple rosbag files sequentially.
- ✅ **IMU Data Extraction** – Reads `/imu/data` topic and computes acceleration magnitude.
- ✅ **Real-time Plotting** – Uses Matplotlib to display IMU vibration patterns dynamically.
- ✅ **Sliding Window Processing** – Maintains a fixed-size window (default: 500 samples).
- ✅ **Timeout Monitoring** – Detects if rosbag playback becomes inactive and stops gracefully.
- ✅ **Multi-threaded Execution** – Runs rosbag playback, data reading, and monitoring in separate threads.
- ✅ **Clean Shutdown** – Handles SIGINT (`Ctrl+C`) to terminate rosbag playback safely.

---

## 🛠 Installation & Setup

### **1. Prerequisites**
Ensure you have the following installed:
- **ROS (Robot Operating System)** – Noetic, Melodic, or another compatible version.
- **Python 3** with required dependencies:
  ```bash
  pip install matplotlib rospy rosbag
  ```

### **2. Clone the Repository**
```bash
cd ~/catkin_ws/src
git clone https://github.com/your-username/imubag_data.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### **3. Make Script Executable**
```bash
chmod +x ~/catkin_ws/src/imubag_data/scripts/rosbag_playing.py
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
rosrun imubag_data rosbag_playing.py
```

### **Select Rosbag Files**
The script will prompt you to **select one or more rosbag files** using a file dialog.

### **ROS Parameters (Optional)**
Set rosbag path and IMU topic before running:
```bash
rosparam set /target_bag "/path/to/your/rosbag.bag"
rosparam set /topic_name "/imu/data"
```

---

## 🏗 How It Works
1. **Select Rosbag Files** – Opens a file dialog for choosing `.bag` files.
2. **Play Rosbags** – Sequentially plays selected bags.
3. **Extract IMU Data** – Reads acceleration from `/imu/data` and computes magnitude.
4. **Plot Data** – Displays a real-time graph of IMU vibration patterns.
5. **Monitor Timeout** – Stops execution if no new data is received within 5 seconds.
6. **Handle Shutdown** – Cleans up processes on `Ctrl+C`.


  ```





