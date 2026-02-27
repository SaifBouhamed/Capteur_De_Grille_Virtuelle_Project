#!/bin/bash

export ROS_DOMAIN_ID=10
source /opt/ros/humble/setup.bash
source ~/Turtlebot_gestes-main/install/setup.bash

cd ~/Turtlebot_gestes-main/Turtlebot3_gestes/turtlebot3_gazebo/worlds
python3 changer_images.py
