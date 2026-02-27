#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
cd "$HOME/Turtlebot_project"
source install/setup.bash

ros2 launch ground_segmentation system_complet.launch.py \
  extra_pitch_deg:=19.0 \
  z_offset:=0.13 \
  max_hz:=5.0 \
  use_packet_stamp:=true
