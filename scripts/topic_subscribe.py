#!/usr/bin/env python3
import rospy
import math
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sensor_msgs.msg import Imu
from collections import deque
import tf.transformations as tft

# Constants
WINDOW_SIZE = 512  # Number of samples in the sliding window
SAMPLING_RATE = 200  # Sampling rate in Hz
OVERLAP = 206 

# =======================
# Buffers for Zed IMU
# =======================
times = deque(maxlen=WINDOW_SIZE)
magnitudes = deque(maxlen=WINDOW_SIZE)
x_data = deque(maxlen=WINDOW_SIZE)
y_data = deque(maxlen=WINDOW_SIZE)
z_data = deque(maxlen=WINDOW_SIZE)
roll_data = deque(maxlen=WINDOW_SIZE)
pitch_data = deque(maxlen=WINDOW_SIZE)
yaw_data = deque(maxlen=WINDOW_SIZE)

# Cached rotation data for Zed IMU
cached_rotation_matrix = None
cached_quat = None

# =======================
# Buffers for Ellipse Imu (Second Sensor)
# =======================
times_zed = deque(maxlen=WINDOW_SIZE)
magnitudes_zed = deque(maxlen=WINDOW_SIZE)
x_data_zed = deque(maxlen=WINDOW_SIZE)
y_data_zed = deque(maxlen=WINDOW_SIZE)
z_data_zed = deque(maxlen=WINDOW_SIZE)
roll_data_zed = deque(maxlen=WINDOW_SIZE)
pitch_data_zed = deque(maxlen=WINDOW_SIZE)
yaw_data_zed = deque(maxlen=WINDOW_SIZE)

# Cached rotation data for Ellipse Imu
cached_rotation_matrix_zed = None
cached_quat_zed = None

data_lock = threading.Lock()

# =======================
# Create Figures and Axes
# =======================
# Figure 1: Overall acceleration (time-domain, FFT, RMS/Peak-to-Peak)
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))
fig.suptitle("IMU Vibration Analysis (Overall)")

# Figure 2: FFT for individual acceleration axes
fig2, (ax_x, ax_y, ax_z) = plt.subplots(3, 1, figsize=(10, 10))
fig2.suptitle("FFT for Individual Acceleration Axes")

# Figure 3: FFT for orientation (Euler angles)
fig3, (ax_roll, ax_pitch, ax_yaw) = plt.subplots(3, 1, figsize=(10, 10))
fig3.suptitle("FFT for Orientation (Euler Angles)")

def imu_callback(msg):
    global cached_rotation_matrix, cached_quat
    with data_lock:
        # Read raw acceleration and orientation
        ax_val, ay_val, az_val = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        quat = [qx, qy, qz, qw]
        euler = tft.euler_from_quaternion(quat)
        roll, pitch, yaw = euler
        roll_data.append(roll)
        pitch_data.append(pitch)
        yaw_data.append(yaw)

        # Compute gravity dynamically based on orientation
        if quat != cached_quat:
            cached_rotation_matrix = tft.quaternion_matrix(quat)
            cached_quat = quat

        # Compute gravity vector in sensor frame
        gravity = cached_rotation_matrix.dot(np.array([0, 0, -9.81, 0]))

        # Original acceleration vector in homogeneous coordinates
        accel_vector = np.array([ax_val, ay_val, az_val, 0])

        # Rotate acceleration to world frame and remove gravity
        accel_world = cached_rotation_matrix.dot(accel_vector)
        accel_world_no_g = accel_world - gravity

        # Rotate back to sensor frame to obtain gravity-compensated values
        inv_rotation_matrix = cached_rotation_matrix.T  # inverse of rotation_matrix
        accel_sensor_no_g = inv_rotation_matrix.dot(accel_world_no_g)

        # Compute overall magnitude of the filtered acceleration (x, y, z)
        magnitude_no_g = math.sqrt(sum(accel_sensor_no_g[i]**2 for i in range(3)))

        # Append data to deques
        times.append(msg.header.stamp.to_sec())
        magnitudes.append(magnitude_no_g)
        x_data.append(accel_sensor_no_g[0])
        y_data.append(accel_sensor_no_g[1])
        z_data.append(accel_sensor_no_g[2])

def imu_callback_zed(msg):
    with data_lock:
        # Read IMU acceleration
        ax_val, ay_val, az_val = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w

        # Append raw data directly to buffers
        times_zed.append(msg.header.stamp.to_sec())
        x_data_zed.append(ax_val)
        y_data_zed.append(ay_val)
        z_data_zed.append(az_val)

        # Store orientation data
        roll, pitch, yaw = tft.euler_from_quaternion([qx, qy, qz, qw])
        roll_data_zed.append(roll)
        pitch_data_zed.append(pitch)
        yaw_data_zed.append(yaw)

        # Compute raw acceleration magnitude
        magnitude = np.sqrt(ax_val**2 + ay_val**2 + az_val**2)
        magnitudes_zed.append(magnitude)

        # rospy.loginfo(f"ZED IMU Raw Accel: [{ax_val}, {ay_val}, {az_val}]")
        # rospy.loginfo(f"ZED IMU Orientation (Roll, Pitch, Yaw): [{roll}, {pitch}, {yaw}]")



# Helper Functions
# ===================================
def apply_hanning_window(signal):
    return signal * np.hanning(len(signal))

def compute_sliding_window_fft(data_array):
    """General FFT computation for a given 1D numpy array."""
    if len(data_array) < WINDOW_SIZE:
        return [], []
    data = np.array(data_array)
    data -= np.mean(data)
    windowed_data = apply_hanning_window(data)
    fft_values = np.abs(np.fft.fft(windowed_data))[:WINDOW_SIZE // 2] * (2.0 / WINDOW_SIZE)
    freq = np.fft.fftfreq(WINDOW_SIZE, d=1.0 / SAMPLING_RATE)[:WINDOW_SIZE // 2]
    return freq, fft_values

def compute_rms(data_array):
    return np.sqrt(np.mean(np.array(data_array) ** 2)) if data_array else 0

def compute_peak_to_peak(data_array):
    return max(data_array) - min(data_array) if data_array else 0

# ===================================
# Plot Update Functions
# ===================================
def update_plot_overall(i):
    with data_lock:
        # Ensure we have some data for both sensors
        if len(times) < 2 or len(times_zed) < 2:
            return

        # Compute relative times for better comparison
        times_rel = [t - times[0] for t in times]
        times_rel_zed = [t - times_zed[0] for t in times_zed]

        # Compute FFTs for overall magnitude
        freq, fft_values = compute_sliding_window_fft(magnitudes)
        freq_zed, fft_values_zed = compute_sliding_window_fft(magnitudes_zed)

        # Compute RMS and Peak-to-Peak for both sensors
        rms_value = compute_rms(magnitudes)
        rms_value_zed = compute_rms(magnitudes_zed)
        peak_to_peak = compute_peak_to_peak(magnitudes)
        peak_to_peak_zed = compute_peak_to_peak(magnitudes_zed)

        # --- Time-domain Plot ---
        ax1.clear()
        ax1.plot(times_rel, magnitudes, color="blue", label="Ellipse IMU")
        ax1.plot(times_rel_zed, magnitudes_zed, color="orange", label="zed Imu")
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Magnitude (m/s²)")
        ax1.set_title("Acceleration Magnitude (Filtered Data)")
        ax1.legend()
        ax1.grid(True)
        max_val = max(max(magnitudes, default=0), max(magnitudes_zed, default=0))
        ax1.set_ylim(0, max_val + 1)

        # --- FFT Plot ---
        ax2.clear()
        if len(freq) > 0 and len(freq_zed) > 0:
            ax2.plot(freq, fft_values, color="purple", label="Ellipse IMU")
            ax2.plot(freq_zed, fft_values_zed, color="brown", label="Zed Imu")
            ax2.set_xlabel("Frequency (Hz)")
            ax2.set_ylabel("Magnitude (m/s²)")
            ax2.set_title("Sliding Window FFT of Overall Vibration")
            ax2.legend()
            ax2.grid(True)
            ax2.set_xlim(0, 100)
            ax2.set_yscale('log')
            max_fft = max(max(fft_values, default=1), max(fft_values_zed, default=1))
            ax2.set_ylim(1e-3, max_fft * 1.2)

        # --- Bar Chart (RMS & Peak-to-Peak) ---
        ax3.clear()
        labels = ["RMS", "Peak-to-Peak"]
        front_values = [rms_value_zed, peak_to_peak_zed]
        imu_values = [rms_value, peak_to_peak]
        x = np.arange(len(labels))
        width = 0.35
        ax3.bar(x - width/2, front_values, width, color="green", label="Zed IMU")
        ax3.bar(x + width/2, imu_values, width, color="red", label="Ellipse Imu")
        ax3.set_xticks(x)
        ax3.set_xticklabels(labels)
        ax3.set_title("RMS & Peak-to-Peak Vibration")
        ax3.legend()
        ax3.grid(True)
        ax3.set_ylim(0, 15)

def update_plot_individual(i):
    with data_lock:
        # Ensure we have enough data for FFT on all axes for both sensors
        if (len(x_data) < WINDOW_SIZE or len(y_data) < WINDOW_SIZE or len(z_data) < WINDOW_SIZE or
            len(x_data_zed) < WINDOW_SIZE or len(y_data_zed) < WINDOW_SIZE or len(z_data_zed) < WINDOW_SIZE):
            return

        # Compute FFT for each axis (Zed IMU)
        freq_x, fft_x = compute_sliding_window_fft(x_data)
        freq_y, fft_y = compute_sliding_window_fft(y_data)
        freq_z, fft_z = compute_sliding_window_fft(z_data)
        # Compute FFT for each axis (Ellipse Imu)
        freq_x_zed, fft_x_zed = compute_sliding_window_fft(x_data_zed)
        freq_y_zed, fft_y_zed = compute_sliding_window_fft(y_data_zed)
        freq_z_zed, fft_z_zed = compute_sliding_window_fft(z_data_zed)

        # --- FFT Plot for X-axis ---
        ax_x.clear()
        if len(freq_x) > 0 and len(freq_x_zed) > 0:
            ax_x.plot(freq_x, fft_x, color="blue", label="Zed IMU")
            ax_x.plot(freq_x_zed, fft_x_zed, color="green", label="Ellipse Imu")
            ax_x.set_xlabel("Frequency (Hz)")
            ax_x.set_ylabel("Magnitude (m/s²)")
            ax_x.set_title("FFT of X-axis")
            ax_x.legend()
            ax_x.grid(True)
            ax_x.set_xlim(0, 100)
            ax_x.set_yscale('log')
            max_val = max(max(fft_x, default=1), max(fft_x_zed, default=1))
            ax_x.set_ylim(1e-3, max_val * 1.2)

        # --- FFT Plot for Y-axis ---
        ax_y.clear()
        if len(freq_y) > 0 and len(freq_y_zed) > 0:
            ax_y.plot(freq_y, fft_y, color="blue", label="Zed IMU")
            ax_y.plot(freq_y_zed, fft_y_zed, color="green", label="Ellipse Imu")
            ax_y.set_xlabel("Frequency (Hz)")
            ax_y.set_ylabel("Magnitude (m/s²)")
            ax_y.set_title("FFT of Y-axis")
            ax_y.legend()
            ax_y.grid(True)
            ax_y.set_xlim(0, 100)
            ax_y.set_yscale('log')
            max_val = max(max(fft_y, default=1), max(fft_y_zed, default=1))
            ax_y.set_ylim(1e-3, max_val * 1.2)

        # --- FFT Plot for Z-axis ---
        ax_z.clear()
        if len(freq_z) > 0 and len(freq_z_zed) > 0:
            ax_z.plot(freq_z, fft_z, color="blue", label="Zed IMU")
            ax_z.plot(freq_z_zed, fft_z_zed, color="green", label="Ellipse Imu")
            ax_z.set_xlabel("Frequency (Hz)")
            ax_z.set_ylabel("Magnitude (m/s²)")
            ax_z.set_title("FFT of Z-axis")
            ax_z.legend()
            ax_z.grid(True)
            ax_z.set_xlim(0, 100)
            ax_z.set_yscale('log')
            max_val = max(max(fft_z, default=1), max(fft_z_zed, default=1))
            ax_z.set_ylim(1e-3, max_val * 1.2)

# def update_plot_orientation(i):
#     with data_lock:
#         # Ensure we have enough data for orientation FFT for both sensors
#         if (len(roll_data) < WINDOW_SIZE or len(pitch_data) < WINDOW_SIZE or len(yaw_data) < WINDOW_SIZE or
#             len(roll_data_zed) < WINDOW_SIZE or len(pitch_data_zed) < WINDOW_SIZE or len(yaw_data_zed) < WINDOW_SIZE):
#             return

#         # Compute FFT for each Euler angle (Zed IMU)
#         freq_roll, fft_roll = compute_sliding_window_fft(roll_data)
#         freq_pitch, fft_pitch = compute_sliding_window_fft(pitch_data)
#         freq_yaw, fft_yaw = compute_sliding_window_fft(yaw_data)
#         # Compute FFT for each Euler angle (Ellipse Imu)
#         freq_roll_zed, fft_roll_zed = compute_sliding_window_fft(roll_data_zed)
#         freq_pitch_zed, fft_pitch_zed = compute_sliding_window_fft(pitch_data_zed)
#         freq_yaw_zed, fft_yaw_zed = compute_sliding_window_fft(yaw_data_zed)

#         # --- FFT Plot for Roll ---
#         ax_roll.clear()
#         if len(freq_roll) > 0 and len(freq_roll_zed) > 0:
#             ax_roll.plot(freq_roll, fft_roll, color="magenta", label=" ZED IMU")
#             ax_roll.plot(freq_roll_zed, fft_roll_zed, color="purple", label=" Ellipse IMu")
#             ax_roll.set_xlabel("Frequency (Hz)")
#             ax_roll.set_ylabel("Magnitude")
#             ax_roll.set_title("FFT of Roll")
#             ax_roll.legend()
#             ax_roll.grid(True)
#             ax_roll.set_xlim(0, 200)
#             ax_roll.set_yscale('log')
#             max_val = max(max(fft_roll, default=1), max(fft_roll_zed, default=1))
#             ax_roll.set_ylim(1e-3, max_val * 1.2)

#         # --- FFT Plot for Pitch ---
#         ax_pitch.clear()
#         if len(freq_pitch) > 0 and len(freq_pitch_zed) > 0:
#             ax_pitch.plot(freq_pitch, fft_pitch, color="orange", label="Zed IMU")
#             ax_pitch.plot(freq_pitch_zed, fft_pitch_zed, color="darkorange", label="Ellipse Imu")
#             ax_pitch.set_xlabel("Frequency (Hz)")
#             ax_pitch.set_ylabel("Magnitude")
#             ax_pitch.set_title("FFT of Pitch")
#             ax_pitch.legend()
#             ax_pitch.grid(True)
#             ax_pitch.set_xlim(0, 200)
#             ax_pitch.set_yscale('log')
#             max_val = max(max(fft_pitch, default=1), max(fft_pitch_zed, default=1))
#             ax_pitch.set_ylim(1e-3, max_val * 1.2)

#         # --- FFT Plot for Yaw ---
#         ax_yaw.clear()
#         if len(freq_yaw) > 0 and len(freq_yaw_zed) > 0:
#             ax_yaw.plot(freq_yaw, fft_yaw, color="cyan", label="Zed IMU")
#             ax_yaw.plot(freq_yaw_zed, fft_yaw_zed, color="teal", label="Ellipse Imu")
#             ax_yaw.set_xlabel("Frequency (Hz)")
#             ax_yaw.set_ylabel("Magnitude")
#             ax_yaw.set_title("FFT of Yaw")
#             ax_yaw.legend()
#             ax_yaw.grid(True)
#             ax_yaw.set_xlim(0, 200)
#             ax_yaw.set_yscale('log')
#             max_val = max(max(fft_yaw, default=1), max(fft_yaw_zed, default=1))
#             ax_yaw.set_ylim(1e-3, max_val * 1.2)

# ===================================
# ROS Subscribers and Animation Setup
# ===================================
rospy.init_node('imu_vibration_analysis', anonymous=True)
rospy.Subscriber("/front_zed/imu_data", Imu, imu_callback_zed)
rospy.Subscriber("/imu/data", Imu, imu_callback)

# Animation for overall plots (acceleration time domain, FFT, RMS/Peak-to-Peak)
ani_overall = animation.FuncAnimation(fig, update_plot_overall, interval=100)
# Animation for individual acceleration axis FFT plots
ani_individual = animation.FuncAnimation(fig2, update_plot_individual, interval=100)
# Animation for orientation FFT plots
# ani_orientation = animation.FuncAnimation(fig3, update_plot_orientation, interval=100)

plt.tight_layout()
plt.show()
rospy.spin()