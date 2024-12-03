#!/bin/bash

# Set up the ROS 2 environment
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash


# Start RealSense_node in a new tab
gnome-terminal --tab --title="RealSense_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run Robowarepkg RealSense_node; exec bash"

# Start web_socket_node in a new tab
gnome-terminal --tab --title="web_socket_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run Robowarepkg web_socket_node; exec bash"

# Start Roboware_node in a new tab
gnome-terminal --tab --title="Roboware_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run Robowarepkg Roboware_node; exec bash"

# Start PID_node in a new tab
gnome-terminal --tab --title="PID_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run Robowarepkg PID_node; exec bash"

# Start serial_read_node in a new tab
gnome-terminal --tab --title="serial_read_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run Robowarepkg serial_read_node; exec bash"

# Start serial_send_node in a new tab
gnome-terminal --tab --title="serial_send_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run Robowarepkg serial_send_node; exec bash"
