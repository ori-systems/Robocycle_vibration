#!/usr/bin/env python3
import rospy
import rosbag
import subprocess
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sensor_msgs.msg import Imu
import math
import time
import os
import signal
import sys
from tkinter import Tk
from tkinter.filedialog import askopenfilenames
from collections import deque

# Constants
WINDOW_SIZE = 500  
STEP_SIZE = 50     
TIMEOUT_SECONDS = 5  

times = deque(maxlen=WINDOW_SIZE)
magnitudes = deque(maxlen=WINDOW_SIZE)  

# Threading and process management
data_lock = threading.Lock()
rosbag_processes = []
play_thread = None
read_thread = None
monitor_thread = None  
rosbag_active = False  
last_data_time = None  

# Initialize Matplotlib figure
fig, ax = plt.subplots()
ax.set_facecolor("white")

def read_rosbag_topic(target_bag, topic_name):
    global rosbag_active, last_data_time

    if not os.path.exists(target_bag):
        rospy.logerr(f"Rosbag file not found: {target_bag}")
        rosbag_active = False
        return  

    bag = rosbag.Bag(target_bag)
    prev_time = None
    counter = 0  

    rospy.loginfo("Started reading rosbag...")

    try:
        for topic, msg, b in bag.read_messages(topics=[topic_name]):
            if rospy.is_shutdown():
                break

            with data_lock: 
                times.append(msg.header.stamp.to_sec())
                magnitude = math.sqrt(msg.linear_acceleration.x**2 + 
                                      msg.linear_acceleration.y**2 + 
                                      msg.linear_acceleration.z**2)
                magnitudes.append(magnitude)

                last_data_time = rospy.Time.now()  
                rosbag_active = True  
                counter += 1  

            curr_time = msg.header.stamp.to_sec()
            if prev_time is not None:
                sleep_time = curr_time - prev_time
                time.sleep(sleep_time) 
            prev_time = curr_time       
            
            # if counter >= STEP_SIZE:
            #     counter = 0  
            #     update_plot()

        rospy.loginfo("Rosbag playback finished.")
        rosbag_active = False  

    finally:
        bag.close()

def play_rosbags(bag_files):
    """Plays multiple rosbag files sequentially."""
    global rosbag_processes, rosbag_active
    processes = []

    for bag_file in bag_files:
        if os.path.exists(bag_file):
            rospy.loginfo(f"Playing rosbag: {bag_file}")
            process = subprocess.Popen(["rosbag", "play", bag_file])
            processes.append(process)
            rosbag_processes.append(process)
        else:
            rospy.logerr(f"Rosbag file does not exist: {bag_file}")
    
    for process in processes:
        process.wait()

    rospy.loginfo("All rosbag playbacks finished.")
    rosbag_active = False  

def update_plot():
    """Updates the Matplotlib plot with the latest IMU data."""
    with data_lock:
        if not times or not magnitudes:  
            return  

    ax.clear()
    ax.set_facecolor("white") 

    times_rel = [t - times[0] for t in times]  

    ax.plot(times_rel, list(magnitudes), label="Vibration Magnitude", color="blue")  
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration Magnitude")
    ax.set_title("IMU Vibration Data (Sliding Window)")
    ax.legend()
    ax.grid(True)
    fig.canvas.draw_idle()  

def monitor_rosbag_timeout():
    global rosbag_active
    rospy.loginfo("Started rosbag timeout monitor...")

    while rosbag_active:
        if last_data_time and (rospy.Time.now() - last_data_time).to_sec() > TIMEOUT_SECONDS:
            rospy.logwarn("Rosbag timed out! Stopping animation.")
            rosbag_active = False
            if plt.fignum_exists(fig.number):  
                plt.close(fig)
            return  
        if rospy.is_shutdown():  # Check if ROS is shutting down
            rosbag_active = False
            if plt.fignum_exists(fig.number):  
                plt.close(fig)
            return
        time.sleep(1)

def animate(i):
    global rosbag_active

    if not rosbag_active:
        rospy.loginfo("Rosbag stopped. Closing plot.")
        if plt.fignum_exists(fig.number):  
            plt.close(fig)
        return  

    update_plot()

def terminal_kill(sig, frame):

    rospy.loginfo("Terminating...")

    rospy.signal_shutdown("User requested shutdown")

    for process in rosbag_processes:
        rospy.loginfo(f"Terminating rosbag process {process.pid}")
        process.terminate()
        process.wait()

    if play_thread and play_thread.is_alive():
        play_thread.join(timeout=1)
    
    if read_thread and read_thread.is_alive():
        read_thread.join(timeout=1)

    if monitor_thread and monitor_thread.is_alive():
        monitor_thread.join(timeout=1)

    if plt.fignum_exists(fig.number):  
        plt.close(fig)
    
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('rosbag_vibration', anonymous=True)

    signal.signal(signal.SIGINT, terminal_kill)

    Tk().withdraw()
    bag_files = askopenfilenames()

    target_bag = rospy.get_param('~target_bag', 'bags/2024-11-28-10-56-30-day11_cityloop_normal/log_motion_2024-11-28-10-56-36_0.bag')
    topic_name = rospy.get_param('~topic_name', '/imu/data')

    if not os.path.exists(target_bag):
        rospy.logerr(f"Rosbag file not found: {target_bag}")
        sys.exit(1)

    read_thread = threading.Thread(target=read_rosbag_topic, args=(target_bag, topic_name))
    read_thread.start()

    play_thread = threading.Thread(target=play_rosbags, args=(bag_files,))
    play_thread.start()

    monitor_thread = threading.Thread(target=monitor_rosbag_timeout)
    monitor_thread.start()

    while not rosbag_active:
        rospy.loginfo("Waiting for rosbag to start...")
        time.sleep(1)

    if rosbag_active:
        ani = animation.FuncAnimation(fig, animate, interval=1000, cache_frame_data=False)

        try:
            plt.show()
        except KeyboardInterrupt:
            plt.close(fig)

    rospy.spin()
    play_thread.join()
    read_thread.join()
    monitor_thread.join()
