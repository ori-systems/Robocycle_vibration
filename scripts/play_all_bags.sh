#!/bin/bash

# Enable error handling
set -e

rosbag play $(rospack find vib_imu)/bags/*.bag  --clock  
# & rosbag play $(rospack find vib_imu)/bags/stationary/stationary.bag --clock


# rosbag play $(rospack find vib_imu)/bags/*.bag --clock

wait 


