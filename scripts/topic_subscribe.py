#!/usr/bin/env python3
import rospy
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sensor_msgs.msg import Imu
from collections import deque
from scipy.signal import butter, filtfilt

# Constants
WINDOW_SIZE = 512  # Number of samples in the sliding window
SAMPLING_RATE = 200  # Sampling rate in Hz

# Buffers for Ellipse IMU
times = deque(maxlen=WINDOW_SIZE)
magnitudes = deque(maxlen=WINDOW_SIZE)
x_data = deque(maxlen=WINDOW_SIZE)
y_data = deque(maxlen=WINDOW_SIZE)
z_data = deque(maxlen=WINDOW_SIZE)

# Buffers for Zed IMU
times_zed = deque(maxlen=WINDOW_SIZE)
magnitudes_zed = deque(maxlen=WINDOW_SIZE)
x_data_zed = deque(maxlen=WINDOW_SIZE)
y_data_zed = deque(maxlen=WINDOW_SIZE)
z_data_zed = deque(maxlen=WINDOW_SIZE)

data_lock = threading.Lock()

# Global FFT result variables (Overall FFT)
fft_freq_overall = []
fft_values_overall = []
fft_freq_overall_zed = []
fft_values_overall_zed = []

# Global FFT result variables (Individual Axes - Ellipse)
fft_freq_x = []
fft_values_x = []
fft_freq_y = []
fft_values_y = []
fft_freq_z = []
fft_values_z = []

# Global FFT result variables (Individual Axes - Zed)
fft_freq_x_zed = []
fft_values_x_zed = []
fft_freq_y_zed = []
fft_values_y_zed = []
fft_freq_z_zed = []
fft_values_z_zed = []

# Create Figures and Axes
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))
fig.suptitle("IMU Vibration Analysis (Overall)")

fig2, (ax_x, ax_y, ax_z) = plt.subplots(3, 1, figsize=(10, 10))
fig2.suptitle("FFT for Individual Acceleration Axes (Ellipse & Zed)")

# IMU Callbacks
def imu_callback(msg):
    with data_lock:
        ax_val, ay_val, az_val = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        times.append(msg.header.stamp.to_sec())
        x_data.append(ax_val)
        y_data.append(ay_val)
        z_data.append(az_val)

        if len(x_data) >= WINDOW_SIZE:
            x_vibration = apply_bandpass_filter(apply_highpass_filter(x_data))
            y_vibration = apply_bandpass_filter(apply_highpass_filter(y_data))
            z_vibration = apply_bandpass_filter(apply_highpass_filter(z_data))
            magnitude_vibration = np.sqrt(x_vibration[-1]**2 + y_vibration[-1]**2 + z_vibration[-1]**2)
            magnitudes.append(magnitude_vibration)

def imu_callback_zed(msg):
    with data_lock:
        ax_val, ay_val, az_val = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        times_zed.append(msg.header.stamp.to_sec())
        x_data_zed.append(ax_val)
        y_data_zed.append(ay_val)
        z_data_zed.append(az_val)

        if len(x_data_zed) >= WINDOW_SIZE:
            x_vibration_zed = apply_bandpass_filter(apply_highpass_filter(x_data_zed))
            y_vibration_zed = apply_bandpass_filter(apply_highpass_filter(y_data_zed))
            z_vibration_zed = apply_bandpass_filter(apply_highpass_filter(z_data_zed))
            magnitude_vibration_zed = np.sqrt(x_vibration_zed[-1]**2 + y_vibration_zed[-1]**2 + z_vibration_zed[-1]**2)
            magnitudes_zed.append(magnitude_vibration_zed)

# Filtering Functions
def butter_highpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a

def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a

def apply_highpass_filter(data, cutoff=0.5, fs=SAMPLING_RATE, order=5):
    if len(data) < WINDOW_SIZE:
        return np.array(data)  # Return raw data if not enough samples
    b, a = butter_highpass(cutoff, fs, order)
    return filtfilt(b, a, np.array(data))

def apply_bandpass_filter(data, lowcut=1, highcut=50, fs=SAMPLING_RATE, order=5):
    if len(data) < WINDOW_SIZE:
        return np.array(data)  # Return raw data if not enough samples

    b, a = butter_bandpass(lowcut, highcut, fs, order)
    filtered_data = filtfilt(b, a, np.array(data))

    # Downsample by a factor of 2
    return filtered_data[::2]


def compute_rms(data_array):
    return np.sqrt(np.mean(np.array(data_array) ** 2)) if data_array else 0

def compute_peak_to_peak(data_array):
    return max(data_array) - min(data_array) if data_array else 0




# FFT Analysis (using rfft for efficiency)
def compute_sliding_window_fft(data_array):
    if len(data_array) < WINDOW_SIZE:
        return [], []
    data = np.array(data_array) - np.mean(data_array)
    fft_values = np.abs(np.fft.rfft(data)) * (2.0 / WINDOW_SIZE)
    freq = np.fft.rfftfreq(WINDOW_SIZE, d=1.0 / SAMPLING_RATE)
    return freq, fft_values

# Thread function for FFT computations
def fft_computation_thread():
    global fft_freq_overall, fft_values_overall, fft_freq_overall_zed, fft_values_overall_zed
    global fft_freq_x, fft_values_x, fft_freq_y, fft_values_y, fft_freq_z, fft_values_z
    global fft_freq_x_zed, fft_values_x_zed, fft_freq_y_zed, fft_values_y_zed, fft_freq_z_zed, fft_values_z_zed
    rate = rospy.Rate(5)  # 10 Hz (every 100ms)
    while not rospy.is_shutdown():
        with data_lock:
            # Overall FFT for Ellipse
            if len(magnitudes) >= WINDOW_SIZE:
                fft_freq_overall, fft_values_overall = compute_sliding_window_fft(magnitudes)
            # Overall FFT for Zed
            if len(magnitudes_zed) >= WINDOW_SIZE:
                fft_freq_overall_zed, fft_values_overall_zed = compute_sliding_window_fft(magnitudes_zed)
            # Individual axes FFT (Ellipse)
            if len(x_data) >= WINDOW_SIZE:
                fft_freq_x, fft_values_x = compute_sliding_window_fft(x_data)
            if len(y_data) >= WINDOW_SIZE:
                fft_freq_y, fft_values_y = compute_sliding_window_fft(y_data)
            if len(z_data) >= WINDOW_SIZE:
                fft_freq_z, fft_values_z = compute_sliding_window_fft(z_data)
            # Individual axes FFT (Zed)
            if len(x_data_zed) >= WINDOW_SIZE:
                fft_freq_x_zed, fft_values_x_zed = compute_sliding_window_fft(x_data_zed)
            if len(y_data_zed) >= WINDOW_SIZE:
                fft_freq_y_zed, fft_values_y_zed = compute_sliding_window_fft(y_data_zed)
            if len(z_data_zed) >= WINDOW_SIZE:
                fft_freq_z_zed, fft_values_z_zed = compute_sliding_window_fft(z_data_zed)
        rate.sleep()

# Animation update for overall plot using precomputed FFT data
def update_plot_overall(i):
    with data_lock:
        if len(times) < 2 or len(times_zed) < 2 or len(magnitudes) < 2 or len(magnitudes_zed) < 2:
            return

        # Compute RMS and Peak-to-Peak
        rms_value = compute_rms(magnitudes)
        rms_value_zed = compute_rms(magnitudes_zed)
        peak_to_peak_val = compute_peak_to_peak(magnitudes)
        peak_to_peak_val_zed = compute_peak_to_peak(magnitudes_zed)

        # Prepare time series (relative times)
        min_length = min(len(times), len(magnitudes))
        times_rel = [t - times[0] for t in list(times)[-min_length:]]
        magnitudes_trimmed = list(magnitudes)[-min_length:]

        min_length_zed = min(len(times_zed), len(magnitudes_zed))
        times_rel_zed = [t - times_zed[0] for t in list(times_zed)[-min_length_zed:]]
        magnitudes_zed_trimmed = list(magnitudes_zed)[-min_length_zed:]

        # Use precomputed FFT data
        freq = fft_freq_overall
        fft_vals = fft_values_overall
        freq_zed = fft_freq_overall_zed
        fft_vals_zed = fft_values_overall_zed

    # Plot time series
    ax1.clear()
    ax1.plot(times_rel, magnitudes_trimmed, label="Ellipse IMU", color="blue")
    ax1.plot(times_rel_zed, magnitudes_zed_trimmed, label="Zed IMU", color="orange")
    ax1.set_title("Acceleration Magnitude (Filtered)")
    ax1.legend()
    ax1.grid(True)

    # ✅ Optimize the FFT Plot (Reduce Points Rendered)
    ax2.clear()
    if len(freq) > 0 and len(freq_zed) > 0:
        downsample = 2 
        ax2.plot(freq[::downsample], fft_vals[::downsample], color="purple", label="Ellipse IMU")
        ax2.plot(freq_zed[::downsample], fft_vals_zed[::downsample], color="brown", label="Zed IMU")
        ax2.set_xlabel("Frequency (Hz)")
        ax2.set_ylabel("Magnitude (m/s²)")
        ax2.set_title("Sliding Window FFT of Overall Vibration")
        ax2.legend()
        ax2.grid(True)
        ax2.set_xlim(0, 100)

        # Remove log scale and use linear scale with correct limits
        max_fft = max(max(fft_vals, default=1), max(fft_vals_zed, default=1))
        ax2.set_ylim(0, max_fft * 1.2)

    # Plot bar graph of RMS & Peak-to-Peak
    ax3.clear()
    labels = ["RMS", "Peak-to-Peak"]
    front_values = [rms_value_zed, peak_to_peak_val_zed]
    imu_values = [rms_value, peak_to_peak_val]
    x = np.arange(len(labels))
    width = 0.35
    ax3.bar(x - width/2, front_values, width, color="green", label="Zed IMU")
    ax3.bar(x + width/2, imu_values, width, color="red", label="Ellipse IMU")
    ax3.set_xticks(x)
    ax3.set_xticklabels(labels)
    ax3.set_title("RMS & Peak-to-Peak Vibration")
    ax3.legend()
    ax3.grid(True)
    ax3.set_ylim(0, 2)

# Animation update for individual axis plots using precomputed FFT data
def update_plot_individual(i):
    with data_lock:
        # Use precomputed FFT data for individual axes
        f_x, fft_x = fft_freq_x, fft_values_x
        f_y, fft_y = fft_freq_y, fft_values_y
        f_z, fft_z = fft_freq_z, fft_values_z

        f_x_zed, fft_x_zed = fft_freq_x_zed, fft_values_x_zed
        f_y_zed, fft_y_zed = fft_freq_y_zed, fft_values_y_zed
        f_z_zed, fft_z_zed = fft_freq_z_zed, fft_values_z_zed

    # Plot X-axis FFT
    ax_x.clear()
    if len(f_x) and len(f_x_zed):
        ax_x.plot(f_x, fft_x, color="green", label="Ellipse X-axis")
        ax_x.plot(f_x_zed, fft_x_zed, color="blue", linestyle="dashed", label="Zed X-axis")
    ax_x.set_title("FFT of X-axis")
    ax_x.legend()
    ax_x.grid(True)

    # Plot Y-axis FFT
    ax_y.clear()
    if len(f_y) and len(f_y_zed):
        ax_y.plot(f_y, fft_y, color="green", label="Ellipse Y-axis")
        ax_y.plot(f_y_zed, fft_y_zed, color="blue", linestyle="dashed", label="Zed Y-axis")
    ax_y.set_title("FFT of Y-axis")
    ax_y.legend()
    ax_y.grid(True)

    # Plot Z-axis FFT
    ax_z.clear()
    if len(f_z) and len(f_z_zed):
        ax_z.plot(f_z, fft_z, color="green", label="Ellipse Z-axis")
        ax_z.plot(f_z_zed, fft_z_zed, color="blue", linestyle="dashed", label="Zed Z-axis")
    ax_z.set_title("FFT of Z-axis")
    ax_z.legend()
    ax_z.grid(True)

# Initialize ROS node and subscribers
rospy.init_node('imu_vibration_analysis', anonymous=True)
rospy.Subscriber("/front_zed/imu_data", Imu, imu_callback_zed)
rospy.Subscriber("/imu/data", Imu, imu_callback)

# Start the FFT computation thread
fft_thread = threading.Thread(target=fft_computation_thread)
fft_thread.daemon = True
fft_thread.start()

# Optionally, run rospy.spin() in a separate thread so that plt.show() remains responsive
def ros_spin_thread():
    rospy.spin()

ros_thread = threading.Thread(target=ros_spin_thread)
ros_thread.daemon = True
ros_thread.start()

# Set up animations for the two figures
ani_overall = animation.FuncAnimation(fig, update_plot_overall, interval=100)
# ani_individual = animation.FuncAnimation(fig2, update_plot_individual, interval=100)

plt.tight_layout()
plt.show()
