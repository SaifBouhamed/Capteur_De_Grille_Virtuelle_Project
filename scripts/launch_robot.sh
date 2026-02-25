#!/bin/bash

unset ROS_LOCALHOST_ONLY
unset ROS_DISCOVERY_SERVER
unset ROS_DISCOVERY_SERVER_PORT
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset CYCLONEDDS_URI
unset RMW_IMPLEMENTATION

export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

source /opt/ros/humble/setup.bash
source ~/Turtlebot_gestes-main/install/setup.bash

ros2 daemon stop
ros2 daemon start

ros2 launch robot_supervisor robot_launch.py server_host:=127.0.0.1 server_port:=9900 image_topic:=/rgb_camera/image
