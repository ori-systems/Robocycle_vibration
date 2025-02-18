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
OVERLAP = 206  # Number of samples to overlap between windows 

# Data buffers for overall magnitude (acceleration)
times = deque(maxlen=WINDOW_SIZE)
magnitudes = deque(maxlen=WINDOW_SIZE)

# Data buffers for individual axes (filtered, gravity removed)
x_data = deque(maxlen=WINDOW_SIZE)
y_data = deque(maxlen=WINDOW_SIZE)
z_data = deque(maxlen=WINDOW_SIZE)

# Data buffers for orientation (Euler angles: roll, pitch, yaw)
roll_data = deque(maxlen=WINDOW_SIZE)
pitch_data = deque(maxlen=WINDOW_SIZE)
yaw_data = deque(maxlen=WINDOW_SIZE)

data_lock = threading.Lock()
cached_rotation_matrix = None
cached_quat = None

# Create main figure with subplots for overall acceleration data
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))
fig.suptitle("IMU Vibration Analysis (Overall)")

# Create a separate figure for FFT of individual acceleration axes
fig2, (ax_x, ax_y, ax_z) = plt.subplots(3, 1, figsize=(10, 10))
fig2.suptitle("FFT for Individual Acceleration Axes")

# Create another separate figure for FFT of orientation (Euler angles)
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
        # Cache the rotation matrix if orientation remains unchanged
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

def compute_rms():
    return np.sqrt(np.mean(np.array(magnitudes) ** 2)) if magnitudes else 0

def compute_peak_to_peak():
    return max(magnitudes) - min(magnitudes) if magnitudes else 0

def update_plot_overall(i):
    with data_lock:
        if len(times) < 2:
            return

        times_rel = [t - times[0] for t in times]
        freq, fft_values = compute_sliding_window_fft(magnitudes)
        rms_value = compute_rms()
        peak_to_peak = compute_peak_to_peak()

        # Time-domain Plot for overall magnitude
        ax1.clear()
        ax1.plot(times_rel, magnitudes, color="blue")
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Magnitude (m/s²)")
        ax1.set_title("Acceleration Magnitude (Filtered Data)")
        ax1.grid(True)
        ax1.set_ylim(0, max(magnitudes) + 1)

        # FFT Plot for overall magnitude
        ax2.clear()
        if len(freq) > 0:
            ax2.plot(freq, fft_values, color="purple")
            ax2.set_xlabel("Frequency (Hz)")
            ax2.set_ylabel("Magnitude (m/s²)")
            ax2.set_title("Sliding Window FFT of Overall Vibration")
            ax2.grid(True)
            ax2.set_xlim(0, 100)  # Adjusted for new sampling rate
            ax2.set_yscale('log')
            ax2.set_ylim(1e-3, max(fft_values) * 1.2 if fft_values.any() else 1)

        # Bar Chart (RMS & Peak-to-Peak)
        ax3.clear()
        ax3.bar(["RMS", "Peak-to-Peak"], [rms_value, peak_to_peak], color=["green", "red"])
        ax3.set_title("RMS & Peak-to-Peak Vibration")
        ax3.grid(True)
        ax3.set_ylim(0, max(rms_value, peak_to_peak) + 1)

def update_plot_individual(i):
    with data_lock:
        # Only update if we have enough data for FFT
        if len(x_data) < WINDOW_SIZE or len(y_data) < WINDOW_SIZE or len(z_data) < WINDOW_SIZE:
            return

        # Compute FFT for each acceleration axis
        freq_x, fft_x = compute_sliding_window_fft(x_data)
        freq_y, fft_y = compute_sliding_window_fft(y_data)
        freq_z, fft_z = compute_sliding_window_fft(z_data)

        # FFT Plot for X-axis
        ax_x.clear()
        if len(freq_x) > 0:
            ax_x.plot(freq_x, fft_x, color="red")
            ax_x.set_xlabel("Frequency (Hz)")
            ax_x.set_ylabel("Magnitude (m/s²)")
            ax_x.set_title("FFT of X-axis")
            ax_x.grid(True)
            ax_x.set_xlim(0, 100)
            ax_x.set_yscale('log')
            ax_x.set_ylim(1e-3, max(fft_x) * 1.2 if fft_x.any() else 1)

        # FFT Plot for Y-axis
        ax_y.clear()
        if len(freq_y) > 0:
            ax_y.plot(freq_y, fft_y, color="green")
            ax_y.set_xlabel("Frequency (Hz)")
            ax_y.set_ylabel("Magnitude (m/s²)")
            ax_y.set_title("FFT of Y-axis")
            ax_y.grid(True)
            ax_y.set_xlim(0, 100)
            ax_y.set_yscale('log')
            ax_y.set_ylim(1e-3, max(fft_y) * 1.2 if fft_y.any() else 1)

        # FFT Plot for Z-axis
        ax_z.clear()
        if len(freq_z) > 0:
            ax_z.plot(freq_z, fft_z, color="blue")
            ax_z.set_xlabel("Frequency (Hz)")
            ax_z.set_ylabel("Magnitude (m/s²)")
            ax_z.set_title("FFT of Z-axis")
            ax_z.grid(True)
            ax_z.set_xlim(0, 100)
            ax_z.set_yscale('log')
            ax_z.set_ylim(1e-3, max(fft_z) * 1.2 if fft_z.any() else 1)

def update_plot_orientation(i):
    with data_lock:
        # Ensure we have enough data for orientation FFT
        if len(roll_data) < WINDOW_SIZE or len(pitch_data) < WINDOW_SIZE or len(yaw_data) < WINDOW_SIZE:
            return

        # Compute FFT for each Euler angle
        freq_roll, fft_roll = compute_sliding_window_fft(roll_data)
        freq_pitch, fft_pitch = compute_sliding_window_fft(pitch_data)
        freq_yaw, fft_yaw = compute_sliding_window_fft(yaw_data)

        # FFT Plot for Roll
        ax_roll.clear()
        if len(freq_roll) > 0:
            ax_roll.plot(freq_roll, fft_roll, color="magenta")
            ax_roll.set_xlabel("Frequency (Hz)")
            ax_roll.set_ylabel("Magnitude")
            ax_roll.set_title("FFT of Roll")
            ax_roll.grid(True)
            ax_roll.set_xlim(0, 100)
            ax_roll.set_yscale('log')
            ax_roll.set_ylim(1e-3, max(fft_roll) * 1.2 if fft_roll.any() else 1)

        # FFT Plot for Pitch
        ax_pitch.clear()
        if len(freq_pitch) > 0:
            ax_pitch.plot(freq_pitch, fft_pitch, color="orange")
            ax_pitch.set_xlabel("Frequency (Hz)")
            ax_pitch.set_ylabel("Magnitude")
            ax_pitch.set_title("FFT of Pitch")
            ax_pitch.grid(True)
            ax_pitch.set_xlim(0, 100)
            ax_pitch.set_yscale('log')
            ax_pitch.set_ylim(1e-3, max(fft_pitch) * 1.2 if fft_pitch.any() else 1)

        # FFT Plot for Yaw
        ax_yaw.clear()
        if len(freq_yaw) > 0:
            ax_yaw.plot(freq_yaw, fft_yaw, color="cyan")
            ax_yaw.set_xlabel("Frequency (Hz)")
            ax_yaw.set_ylabel("Magnitude")
            ax_yaw.set_title("FFT of Yaw")
            ax_yaw.grid(True)
            ax_yaw.set_xlim(0, 100)
            ax_yaw.set_yscale('log')
            ax_yaw.set_ylim(1e-3, max(fft_yaw) * 1.2 if fft_yaw.any() else 1)

rospy.init_node('imu_vibration_analysis', anonymous=True)
rospy.Subscriber("/imu/data", Imu, imu_callback)

# Animation for overall plots (acceleration time domain, FFT, RMS/Peak-to-Peak)
ani_overall = animation.FuncAnimation(fig, update_plot_overall, interval=100)
# Animation for individual acceleration axis FFT plots
ani_individual = animation.FuncAnimation(fig2, update_plot_individual, interval=100)
# Animation for orientation FFT plots
ani_orientation = animation.FuncAnimation(fig3, update_plot_orientation, interval=100)

plt.tight_layout()
plt.show()
rospy.spin()
