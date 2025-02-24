#!/usr/bin/env python3

import rospy
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sensor_msgs.msg import Imu
from collections import deque
from scipy.signal import butter, filtfilt

# === Constants ===
WINDOW_SIZE = 512  # Samples in the sliding window
SAMPLING_RATE = 200  # Hz

# === Buffers for Stationary IMU Data ===
stationary_x, stationary_y, stationary_z = [], [], []
stationary_processed = False
stationary_fft_x, stationary_fft_y, stationary_fft_z = None, None, None

# === Buffers for Moving IMU Data ===
x_data = deque(maxlen=WINDOW_SIZE)
y_data = deque(maxlen=WINDOW_SIZE)
z_data = deque(maxlen=WINDOW_SIZE)
magnitudes = deque(maxlen=WINDOW_SIZE)

# Thread Safety
data_lock = threading.Lock()

# === Create Figures and Axes for Real-Time Plotting ===
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
fig.suptitle("IMU Vibration Analysis")

# === Filtering Functions ===
def butter_highpass(cutoff=3, fs=SAMPLING_RATE, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a

def butter_bandpass(lowcut=1, highcut=50, fs=SAMPLING_RATE, order=5):
    nyq = 0.5 * fs
    low, high = lowcut / nyq, highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a

def apply_highpass_filter(data):
    if len(data) < WINDOW_SIZE:
        return np.array(data)
    b, a = butter_highpass()
    return filtfilt(b, a, np.array(data))

def apply_bandpass_filter(data):
    if len(data) < WINDOW_SIZE:
        return np.array(data)
    b, a = butter_bandpass()
    return filtfilt(b, a, np.array(data))

# === FFT Computation ===
def compute_fft(data):
    data = np.array(data) - np.mean(data)  # Remove DC component
    window = np.hanning(len(data))
    data *= window
    fft_values = np.abs(np.fft.rfft(data)) / len(data)  # Normalize FFT
    freqs = np.fft.rfftfreq(len(data), d=1.0 / SAMPLING_RATE)
    return freqs, fft_values

# === IMU Callbacks ===
def imu_callback_stationary(msg):
    """ Processes stationary IMU data to establish baseline noise. """
    global stationary_processed, stationary_fft_x, stationary_fft_y, stationary_fft_z
    if not stationary_processed:
        stationary_x.append(msg.linear_acceleration.x)
        stationary_y.append(msg.linear_acceleration.y)
        stationary_z.append(msg.linear_acceleration.z)

        if len(stationary_x) >= WINDOW_SIZE:
            filtered_x = apply_bandpass_filter(apply_highpass_filter(stationary_x))
            filtered_y = apply_bandpass_filter(apply_highpass_filter(stationary_y))
            filtered_z = apply_bandpass_filter(apply_highpass_filter(stationary_z))

            _, stationary_fft_x = compute_fft(filtered_x)
            _, stationary_fft_y = compute_fft(filtered_y)
            _, stationary_fft_z = compute_fft(filtered_z)

            stationary_processed = True
            rospy.loginfo("Stationary IMU Processing Complete.")

def imu_callback_moving(msg):
    """ Processes moving IMU data and extracts vibration signals. """
    global stationary_processed, stationary_fft_x, stationary_fft_y, stationary_fft_z
    if not stationary_processed:
        return
    
    with data_lock:
        x_data.append(msg.linear_acceleration.x)
        y_data.append(msg.linear_acceleration.y)
        z_data.append(msg.linear_acceleration.z)

        if len(x_data) >= WINDOW_SIZE:
            filtered_x = apply_bandpass_filter(apply_highpass_filter(x_data))
            filtered_y = apply_bandpass_filter(apply_highpass_filter(y_data))
            filtered_z = apply_bandpass_filter(apply_highpass_filter(z_data))

            freqs, moving_fft_x = compute_fft(filtered_x)
            _, moving_fft_y = compute_fft(filtered_y)
            _, moving_fft_z = compute_fft(filtered_z)

            # Fix: Use division instead of subtraction to avoid over-reducing FFT
            vibration_x = moving_fft_x / (stationary_fft_x + 1e-6)
            vibration_y = moving_fft_y / (stationary_fft_y + 1e-6)
            vibration_z = moving_fft_z / (stationary_fft_z + 1e-6)

            # Fix: Compute RMS for better vibration magnitude estimation
            def compute_rms(data):
                return np.sqrt(np.mean(np.square(data)))

            magnitude_vibration = compute_rms(np.array([vibration_x, vibration_y, vibration_z]))
            magnitudes.append(magnitude_vibration)

# === Real-Time Plotting Function ===
def update_plot(i):
    """ Updates the vibration magnitude and FFT plots in real-time. """
    with data_lock:
        if len(magnitudes) < 2:
            return
        
        ax1.clear()
        ax1.plot(range(len(magnitudes)), list(magnitudes), label="Vibration Magnitude")
        ax1.set_title("Filtered IMU Vibration Magnitude Over Time")
        ax1.set_xlabel("Time Steps")
        ax1.set_ylabel("Magnitude (m/s²)")
        ax1.legend()
        ax1.grid(True)

        if stationary_processed:
            ax2.clear()
            freqs, fft_values = compute_fft(list(magnitudes))
            # Fix: Avoid log(0) errors in FFT plotting
            ax2.plot(freqs, np.log1p(fft_values), label="FFT of Vibrations")
            ax2.set_title("FFT of Vibration Data")
            ax2.set_xlabel("Frequency (Hz)")
            ax2.set_ylabel(" Amplitude")
            ax2.legend()
            ax2.grid(True)

# === ROS Initialization ===
rospy.init_node('imu_vibration_analysis', anonymous=True)
rospy.Subscriber("/stationary_imu/data", Imu, imu_callback_stationary)
rospy.Subscriber("/imu/data", Imu, imu_callback_moving)

# Start Animation & ROS Spin
ani = animation.FuncAnimation(fig, update_plot, interval=100)
plt.show()
rospy.spin()
