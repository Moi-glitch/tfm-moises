#!/usr/bin/env bash

# Source ROS 2 and user workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/turtlebot3_ws/install/setup.bash

# Ready for simulation: user can launch Gazebo or RViz as needed
exec "$@"
