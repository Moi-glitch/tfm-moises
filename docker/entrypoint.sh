#!/usr/bin/env bash

set -e

# Source ROS 2 and the built workspace if present
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f /opt/turtlebot3_ws/install/setup.bash ]; then
  source /opt/turtlebot3_ws/install/setup.bash
fi

# Hand off to whatever the user specified
exec "$@"
