#!/bin/bash

# Source ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash

# Source the local workspace
source /app/src/ros2_ws/install/local_setup.bash

ros2 launch hexapod_manager user_controlled.yml