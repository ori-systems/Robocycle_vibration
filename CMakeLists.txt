cmake_minimum_required(VERSION 3.0.2)
project(vib_imu)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  rosbag
)

catkin_package()

# install(PROGRAMS
#   scripts/rosbag_parsing.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
