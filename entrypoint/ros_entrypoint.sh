#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/autoware_ws/install/setup.bash


ros2 run autoware_v2x_cam_publisher cam_publisher_node --ros-args --params-file ./src/autoware_v2v_cam_publisher_node/params.yaml --log-level DEBUG &


ros2 launch etsi_its_conversion converter.launch.py

ros2 launch udp_driver udp_sender_launch.py

tail -f /dev/null
