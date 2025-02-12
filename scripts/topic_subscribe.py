#!/usr/bin/env python3
import rospy
import math
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sensor_msgs.msg import Imu
from collections import deque
from numpy.fft import fft

# Constants
WINDOW_SIZE = 1024  # Recommended for real-time + good frequency resolution
SAMPLING_RATE = 200  # IMU sampling rate in Hz
OVERLAP = 512  # 50% overlap for sliding window
CUTOFF_FREQ_LOW = 0.5  # Low cutoff frequency for filtering body movement
CUTOFF_FREQ_HIGH = 50  # High cutoff frequency to remove irrelevant noise

# Data buffers
times = deque(maxlen=WINDOW_SIZE)
magnitudes = deque(maxlen=WINDOW_SIZE)
data_lock = threading.Lock()

# Create a single figure with subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))
fig.suptitle("IMU Vibration Analysis")

def apply_hanning_window(signal):
    """Apply a Hanning window to reduce spectral leakage before FFT."""
    return signal * np.hanning(len(signal))

def compute_sliding_window_fft():
    """Compute FFT using a sliding window approach with Hanning windowing."""
    if len(magnitudes) < WINDOW_SIZE:
        return [], []
    
    data = np.array(magnitudes)
    data = data - np.mean(data)

    num_segments = (len(data) - OVERLAP) // (WINDOW_SIZE - OVERLAP)
    fft_avg = np.zeros(WINDOW_SIZE // 2)
    
    for i in range(num_segments):
        start = i * (WINDOW_SIZE - OVERLAP)
        segment = data[start:start + WINDOW_SIZE]
        windowed_segment = apply_hanning_window(segment)
        
        freq = np.fft.fftfreq(WINDOW_SIZE, d=1.0 / SAMPLING_RATE)[:WINDOW_SIZE // 2]
        fft_values = np.abs(np.fft.fft(windowed_segment))[:WINDOW_SIZE // 2] / WINDOW_SIZE  
        fft_avg += fft_values
    
    fft_avg /= num_segments 
    return freq, fft_avg

def imu_callback(msg):
    """Callback function for the IMU topic."""
    with data_lock:
        timestamp = msg.header.stamp.to_sec()
        magnitude = math.sqrt(msg.linear_acceleration.x**2 + 
                              msg.linear_acceleration.y**2 + 
                              msg.linear_acceleration.z**2)
        times.append(timestamp)
        magnitudes.append(magnitude)

def compute_rms():
    """Computes rolling RMS of the vibration magnitude."""
    if len(magnitudes) == 0:
        return 0
    return np.sqrt(np.mean(np.array(magnitudes) ** 2))

def compute_peak_to_peak():
    """Computes peak-to-peak vibration magnitude."""
    if len(magnitudes) == 0:
        return 0
    return max(magnitudes) - min(magnitudes)

# Update function for combined plots
def update_plot(i):
    with data_lock:
        if len(times) < 2:
            return

        times_rel = [t - times[0] for t in times]
        freq, fft_values = compute_sliding_window_fft()
        rms_value = compute_rms()
        peak_to_peak = compute_peak_to_peak()

        # Clear axes
        ax1.clear()
        ax2.clear()
        ax3.clear()

        # Plot Acceleration Magnitude
        ax1.plot(times_rel, magnitudes, label="Acceleration Magnitude", color="blue")
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Magnitude")
        ax1.set_title("Acceleration Magnitude (Filtered Data)")
        ax1.grid(True)
        ax1.legend()

        # Plot RMS & Peak-to-Peak
        ax2.bar(["RMS"], [rms_value], color="green")
        ax2.bar(["Peak-to-Peak"], [peak_to_peak], color="red")
        ax2.set_title("RMS & Peak-to-Peak Vibration")
        ax2.set_ylim(0, max(rms_value, peak_to_peak) * 1.2)
        ax2.grid(True)

        # Plot FFT Spectrum
        if len(freq) > 0:
            ax3.plot(freq, fft_values, label=" FFT Spectrum", color="purple")
            ax3.set_xlabel("Frequency (Hz)")
            ax3.set_ylabel("Magnitude")
            ax3.set_title("Sliding Window FFT of IMU Vibration Data")
            ax3.grid(True)
            ax3.legend()

# ROS Node Initialization
rospy.init_node('imu_vibration_analysis', anonymous=True)
rospy.Subscriber("/imu/data", Imu, imu_callback)

# Matplotlib Animation
ani = animation.FuncAnimation(fig, update_plot, interval=100)

plt.tight_layout()
plt.show()
rospy.spin()
