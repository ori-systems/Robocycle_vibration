# IMU Vibration Analysis Using FFT

## Overview
This repository contains a Python-based implementation of **IMU Vibration Analysis** using a **Sliding Window FFT approach**. The system processes real-time acceleration data from an **Inertial Measurement Unit (IMU)**, applies signal processing techniques, and visualizes vibration frequencies to analyze road conditions and cycling dynamics.

The project integrates **stationary and moving IMU data** to separate noise from actual vibrations, enhancing vibration analysis accuracy. It also includes tools for **rosbag parsing and playback** to analyze pre-recorded IMU data.

## Features
✅ **Real-time IMU data acquisition** using ROS  
✅ **Sliding Window FFT** for continuous frequency analysis  
✅ **Hanning windowing & bandpass filtering** to improve frequency representation  
✅ **DC Offset & Noise Removal** for accurate analysis  
✅ **Highpass and Bandpass Filtering** to extract relevant frequency components  
✅ **Integration of stationary IMU for enhanced vibration separation**  
✅ **Rosbag playback & analysis for offline processing**  
✅ **Live visualization of acceleration and frequency spectrum**  

## Installation
### **Prerequisites**
- Python 3.7+
- ROS (Robot Operating System)
- `numpy`
- `matplotlib`
- `rospy`
- `sensor_msgs`
- `scipy`

### **Setup**
```bash
# Clone this repository
git clone https://github.com/your-repo/imu-vibration-analysis.git
cd imu-vibration-analysis

# Install dependencies
pip install numpy matplotlib rospy sensor_msgs scipy
```

## How It Works
### **1. IMU Data Collection**
- The system subscribes to `/imu/data` and `/front_zed/imu_data` topics, extracting **acceleration values (X, Y, Z)**.
- The magnitude of acceleration is computed as:
  
  $$ \text{Magnitude} = \sqrt{x^2 + y^2 + z^2} $$

### **2. Preprocessing**
- **DC Offset Removal:** Eliminates static bias from the sensor:
  
  $$ \text{adjusted} = x - {\frac{1}{N} \sum x} $$
  
- **Highpass Filtering (Cutoff Frequency: 0.5 Hz):** Removes low-frequency drift and slow movements:
  
  $$ H(s) = {\frac{s}{s + \omega_c}} $$
  
- **Bandpass Filtering (Range: 1Hz - 50Hz):** Retains only relevant vibration frequencies:
  
  $$ H(s) = {\frac{s^2}{s^2 + \omega_c/Q s + \omega_c^2}} $$
  
- **Hanning Windowing:** Reduces spectral leakage before FFT computation.

### **3. FFT-Based Vibration Analysis**
- Overlapping **windowed segments** are transformed into the frequency domain.
- The averaged FFT spectrum is computed for more stable visualization.
- The stationary IMU data is used as a reference to filter noise from moving IMU readings.

### **4. Comparison of `topic_subscribe.py` and `stationary_imu.py`**
Both scripts process IMU data and apply signal filtering techniques, but they serve different purposes:

| Feature | `topic_subscribe.py` | `stationary_imu.py` |
|---------|---------------------|---------------------|
| **IMU Sources** | Reads data from a moving IMU | Uses stationary IMU for baseline noise comparison |
| **Filtering** | Applies **highpass (0.5Hz)** and **bandpass (1Hz - 50Hz)** filtering | Applies **bandpass (1Hz - 50Hz)** filtering, adjusting based on stationary noise |
| **Purpose** | Analyzes real-time vibration while moving | Captures stationary IMU data first to subtract noise from moving IMU data |
| **Noise Handling** | Filters high-frequency and low-frequency noise | Uses stationary IMU noise profile to refine vibration separation |
| **Data Processing** | Directly processes live data from `/imu/data` | First processes stationary IMU data, then compares moving data |
| **Output** | Real-time FFT of IMU vibrations | FFT after subtracting stationary noise from moving IMU |

`topic_subscribe.py` is ideal for direct IMU-based vibration analysis, while `stationary_imu.py` improves accuracy by accounting for stationary noise before analyzing movement-based vibrations.

### **5. Filtering Implementation in Different Scripts**
- `topic_subscribe.py`: Uses **highpass (0.5Hz cutoff) and bandpass (1Hz - 50Hz) filtering** to clean acceleration signals before FFT analysis.
- `stationary_imu.py`: Applies **bandpass filtering (1Hz - 50Hz) to stationary IMU data**, where the exact filtering values depend on the stationary IMU's noise characteristics before extracting baseline noise.
- `rosbag_parsing.py`: Filters extracted IMU data from rosbags using **highpass (0.5Hz) and bandpass (1Hz - 50Hz) filters** before plotting vibration results.

### **6. Rosbag Parsing and Playback**
- The system supports **rosbag playback** for recorded IMU data.
- `rosbag_parsing.py` extracts vibration data from a rosbag file and visualizes it.

### **7. Visualization**
- **Acceleration Magnitude Plot:** Displays real-time acceleration magnitude.
- **FFT Spectrum Plot:** Shows dominant vibration frequencies.

## Running the Scripts
### **Live IMU Data Analysis**
```bash
roslaunch imu_vibration_analysis rosbag_play.launch
python topic_subscribe.py
```

### **Stationary vs. Moving IMU Comparison**
```bash
roslaunch imu_vibration_analysis rosbag_play.launch
python stationary_imu.py
```

### **Rosbag Playback and Parsing**
```bash
bash play_all_bags.sh  # Plays all rosbags in the directory
python rosbag_parsing.py  # Parses and visualizes vibration data
```

## Interpreting Results
| **Frequency Range (Hz)** | **Interpretation** |
|-----------------|------------------------------------|
| 0 - 5 Hz       | Cyclist motion, frame flex        |
| 5 - 20 Hz      | Road surface texture variations   |
| 20 - 50 Hz     | Bike component vibrations         |
| 50+ Hz         | High-frequency mechanical noise   |

## Example Output
![FFT Visualization](zed.png,st.png)

## Potential Applications
🚴 **Cycling Comfort Analysis** – Identify road roughness.  
📡 **IMU-Based Diagnostics** – Detect mechanical issues in bikes.  
🔬 **Vibration Research** – Analyze different road conditions.  

## License
This project is licensed under the MIT License.

---
Developed for IMU-based cycling vibration analysis and research.
