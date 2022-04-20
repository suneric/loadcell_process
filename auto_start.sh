#!/bin/bash
sudo -s -H << 'EOF'
export ROS_MASTER_URI=http://192.168.1.19:11311
export ROS_IP=192.168.1.15

source /opt/ros/noetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash

python3 /home/ubuntu/catkin_ws/src/loadcell_process/scripts/load_cell_interface.py
EOF
