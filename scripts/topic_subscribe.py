#!/usr/bin/env python3
import rospy
import math
import time
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sensor_msgs.msg import Imu
from collections import deque

WINDOW_SIZE = 500 

times = deque(maxlen=WINDOW_SIZE)
magnitudes = deque(maxlen=WINDOW_SIZE)

data_lock = threading.Lock()


fig, ax = plt.subplots()
ax.set_facecolor("white")

def imu_callback(msg):
    """Callback function for the IMU topic."""
    with data_lock:
        timestamp = msg.header.stamp.to_sec()
        magnitude = math.sqrt(msg.linear_acceleration.x**2 + 
                              msg.linear_acceleration.y**2 + 
                              msg.linear_acceleration.z**2)
        times.append(timestamp)
        magnitudes.append(magnitude)

        
        if len(times) > WINDOW_SIZE:
            times.popleft()
            magnitudes.popleft()

def update_plot():
    """Updates the Matplotlib plot with the latest IMU data."""
    with data_lock:
        if len(times) < WINDOW_SIZE:
            return

        ax.clear()
        ax.set_facecolor("white")

        times_rel = [t - times[0] for t in times]  
        

        ax.plot(times_rel, magnitudes, label="Vibration Magnitude", color="blue")  
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Acceleration Magnitude")
        ax.set_title(f"IMU Vibration Data (Last {WINDOW_SIZE} Samples)")
        ax.set_ylim(6, 15)
        ax.legend()
        ax.grid(True)
        fig.canvas.draw_idle()


def animate(i):
    """Function for Matplotlib animation."""
    update_plot()

if __name__ == '__main__':
    rospy.init_node('imu_vibration_plotter', anonymous=True)
    imu_topic = rospy.get_param('~imu_topic', '/imu/data')
    
    rospy.Subscriber(imu_topic, Imu, imu_callback)
    
    ani = animation.FuncAnimation(fig, animate, interval=1000, cache_frame_data=False)
    
    try:
        plt.show()
    except rospy.ROSInterruptException:
        pass
