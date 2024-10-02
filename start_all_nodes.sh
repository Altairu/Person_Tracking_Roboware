#!/bin/bash

# Set up the ROS 2 environment
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start web_socket_node in a new tab
gnome-terminal --tab --title="web_socket_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run Robowarepkg web_socket_node; exec bash"

# Start Robowarenode in a new tab
gnome-terminal --tab --title="Robowarenode" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run Robowarepkg Robowarenode; exec bash"

# Start serial_read_node in a new tab
gnome-terminal --tab --title="serial_read_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run Robowarepkg serial_read_node; exec bash"

# Start serial_send_node in a new tab
gnome-terminal --tab --title="serial_send_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run Robowarepkg serial_send_node; exec bash"

# Start position_node in a new tab
gnome-terminal --tab --title="position_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run Robowarepkg position_node; exec bash"