#!/bin/bash
# Exit immediately if a command exits with a non-zero status.
set -e

# Setup the ROS 2 environment by sourcing the master setup file.
source /opt/ros/jazzy/setup.bash

# Check if a local workspace has been built and source it if it exists.
# This allows you to use your custom packages after running 'colcon build'.
if [ -f "/home/rosuser/ros2_ws/install/setup.bash" ]; then
  source /home/rosuser/ros2_ws/install/setup.bash
  echo "Sourced local workspace: /home/rosuser/ros2_ws"
fi

# Execute the command passed into the docker run/exec command.
# This allows you to run 'bash', 'ros2 run ...', etc.
exec "$@"
