# IMU Vibration Analysis Using FFT

## Overview
This repository contains a Python-based implementation of **IMU Vibration Analysis** using a **Sliding Window FFT approach**. It processes real-time acceleration data from an **Inertial Measurement Unit (IMU)**, applies signal processing techniques, and visualizes vibration frequencies to analyze road conditions and cycling dynamics.

## Features
✅ **Real-time IMU data acquisition** using ROS  
✅ **Sliding Window FFT** for continuous frequency analysis  
✅ **Hanning windowing** to reduce spectral leakage  
✅ **DC Offset Removal** for accurate frequency representation  
✅ **Live visualization of acceleration and frequency spectrum**  

## Installation
### **Prerequisites**
- Python 3.7+
- ROS (Robot Operating System)
- `numpy`
- `matplotlib`
- `rospy`
- `sensor_msgs`

### **Setup Instructions**
1. Clone the repository:
   ```sh
   git clone https://github.com/yourusername/imu-vibration-analysis.git
   cd imu-vibration-analysis
   ```
2. Install required dependencies:
   ```sh
   pip install numpy matplotlib rospy sensor_msgs
   ```
3. Ensure your IMU is publishing data on the ROS topic `/imu/data`.
4. Run the script:
   ```sh
   python imu_vibration_analysis.py
   ```

## How It Works
1. **IMU Data Collection:**
   - The script subscribes to `/imu/data`, extracting **acceleration values (X, Y, Z)**.
   - The magnitude of acceleration is computed as:
     
     $$ \text{Magnitude} = \sqrt{x^2 + y^2 + z^2} $$

2. **Preprocessing:**
   - **DC Offset Removal:** Eliminates static bias from the sensor:
     
     $$ x_{\text{adjusted}} = x - \frac{1}{N} \sum x $$
   
   - **Hanning Window:** Reduces spectral leakage before FFT computation.

3. **Sliding Window FFT:**
   - Overlapping **windowed segments** are transformed into the frequency domain.
   - The averaged FFT spectrum is computed for more stable visualization.

4. **Visualization:**
   - **Acceleration Magnitude Plot:** Shows real-time acceleration magnitude.
   - **FFT Spectrum Plot:** Displays dominant vibration frequencies.

## Interpreting Results
| **Frequency Range (Hz)** | **Interpretation** |
|-----------------|------------------------------------|
| 0 - 5 Hz       | Cyclist motion, frame flex        |
| 5 - 20 Hz      | Road surface texture variations   |
| 20 - 50 Hz     | Bike component vibrations         |
| 50+ Hz         | High-frequency mechanical noise   |

## Example Output
![FFT Visualization](example_fft.png)

## Potential Applications
🚴 **Cycling Comfort Analysis** – Identify road roughness.  
📡 **IMU-Based Diagnostics** – Detect mechanical issues in bikes.  
🔬 **Vibration Research** – Analyze different road conditions.  

