#!/usr/bin/env bash
# Install required ROS and system dependencies for all packages in this repository

set -e

# Determine repository root (directory of this script)
REPO_ROOT="$(cd "$(dirname "$0")" && pwd)"

cd "$REPO_ROOT"

# Update rosdep and install dependencies for the packages
rosdep update
rosdep install --from-paths . --ignore-src -y -r --skip-keys ament_python --skip-keys turtlebot3_autorace_camera
