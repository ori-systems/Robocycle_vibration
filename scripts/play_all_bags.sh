#!/bin/bash

# Enable error handling
set -e

rosbag play $(rospack find imuvib_data)/bags/*.bag --clock

wait 


