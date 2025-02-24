#!/usr/bin/env python3

import rospy
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
from sensor_msgs.msg import Imu
from collections import deque
from scipy.signal import butter, filtfilt

# Constants
WINDOW_SIZE = 1024  # Number of samples in the sliding window
SAMPLING_RATE = 200  # Sampling rate in Hz

# Buffers for Stationary IMU Data
stationary_x = []
stationary_y = []
stationary_z = []
stationary_processed = False  # Flag to indicate if processing is done
stationary_fft_x = None
stationary_fft_y = None
stationary_fft_z = None

# Buffers for Moving IMU Data
x_data = deque(maxlen=WINDOW_SIZE)
y_data = deque(maxlen=WINDOW_SIZE)
z_data = deque(maxlen=WINDOW_SIZE)
magnitudes = deque(maxlen=WINDOW_SIZE)

data_lock = threading.Lock()
processing_time = 0  # Timer variable

# Create Figures and Axes
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
fig.suptitle("IMU Vibration Analysis")

def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a

def apply_bandpass_filter(data, lowcut=1, highcut=50, fs=SAMPLING_RATE, order=5):
    if len(data) < WINDOW_SIZE:
        return np.array(data)  # Return raw data if not enough samples
    b, a = butter_bandpass(lowcut, highcut, fs, order)
    return filtfilt(b, a, np.array(data))

def compute_fft(data, fs=SAMPLING_RATE):
    fft_values = np.abs(np.fft.rfft(data)) * (2.0 / WINDOW_SIZE)  # Normalize FFT like second code
    freqs = np.fft.rfftfreq(len(data), d=1.0/fs)
    return freqs, fft_values

def imu_callback_stationary(msg):
    global stationary_processed, stationary_fft_x, stationary_fft_y, stationary_fft_z, processing_time
    if not stationary_processed:
        start_time = time.time()
        stationary_x.append(msg.linear_acceleration.x)
        stationary_y.append(msg.linear_acceleration.y)
        stationary_z.append(msg.linear_acceleration.z)

        if len(stationary_x) >= WINDOW_SIZE:
            filtered_x = apply_bandpass_filter(stationary_x)
            filtered_y = apply_bandpass_filter(stationary_y)
            filtered_z = apply_bandpass_filter(stationary_z)

            _, stationary_fft_x = compute_fft(filtered_x)
            _, stationary_fft_y = compute_fft(filtered_y)
            _, stationary_fft_z = compute_fft(filtered_z)

            stationary_processed = True
            processing_time = time.time() - start_time
            rospy.loginfo(f"Stationary IMU Processing Complete. Time taken: {processing_time:.2f} seconds")

def imu_callback_moving(msg):
    global stationary_processed, stationary_fft_x, stationary_fft_y, stationary_fft_z
    if not stationary_processed:
        return
    
    with data_lock:
        ax_val, ay_val, az_val = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        x_data.append(ax_val)
        y_data.append(ay_val)
        z_data.append(az_val)

        if len(x_data) >= WINDOW_SIZE:
            filtered_x = apply_bandpass_filter(x_data)
            filtered_y = apply_bandpass_filter(y_data)
            filtered_z = apply_bandpass_filter(z_data)

            freqs, moving_fft_x = compute_fft(filtered_x)
            _, moving_fft_y = compute_fft(filtered_y)
            _, moving_fft_z = compute_fft(filtered_z)

            vibration_x = np.maximum(moving_fft_x - stationary_fft_x, 0)
            vibration_y = np.maximum(moving_fft_y - stationary_fft_y, 0)
            vibration_z = np.maximum(moving_fft_z - stationary_fft_z, 0)

            magnitude_vibration = np.sqrt(vibration_x[-1]**2 + vibration_y[-1]**2 + vibration_z[-1]**2)
            magnitudes.append(magnitude_vibration)

def update_plot(i):
    with data_lock:
        if len(magnitudes) < 2:
            return
        
        ax1.clear()
        ax1.plot(range(len(magnitudes)), list(magnitudes), label="Vibration Magnitude")
        ax1.set_title("Filtered IMU Vibration Magnitude Over Time")
        ax1.set_xlabel("Time Steps")
        ax1.set_ylabel("Magnitude")
        ax1.legend()
        ax1.grid(True)
        
        if stationary_processed:
            ax2.clear()
            freqs, fft_values = compute_fft(list(magnitudes))
            ax2.plot(freqs, fft_values, label="FFT of Vibrations")
            ax2.set_title(f"FFT of Vibration Data")
            ax2.set_xlabel("Frequency (Hz)")
            ax2.set_ylabel("Amplitude")
            ax2.legend()
            ax2.grid(True)

ani = animation.FuncAnimation(fig, update_plot, interval=100)

rospy.init_node('imu_vibration_analysis', anonymous=True)
rospy.Subscriber("/stationary_imu/data", Imu, imu_callback_stationary)
rospy.Subscriber("/imu/data", Imu, imu_callback_moving)

plt.show()
rospy.spin()
