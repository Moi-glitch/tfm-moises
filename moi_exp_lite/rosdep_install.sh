#!/usr/bin/env bash
# Install required ROS and system dependencies using rosdep

set -e

# Ensure we run from the package root (where package.xml lives)
if [ ! -f "package.xml" ]; then
  echo "Please run this script from the package root." >&2
  exit 1
fi

# Update rosdep and install dependencies listed in package.xml
rosdep update
rosdep install --from-paths . --ignore-src -y
