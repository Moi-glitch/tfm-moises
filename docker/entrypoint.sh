#!/usr/bin/env bash

set -e

# Allow GUI applications if available
export QT_X11_NO_MITSHM=${QT_X11_NO_MITSHM:-1}
if [ -n "$DISPLAY" ] && command -v xauth >/dev/null; then
  XAUTH_FILE=${XAUTHORITY:-$HOME/.Xauthority}
  if [ ! -f "$XAUTH_FILE" ]; then
    xauth generate "$DISPLAY" . trusted
    export XAUTHORITY="$XAUTH_FILE"
  fi
fi

# Source ROS 2 and the built workspace if present
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f /opt/turtlebot3_ws/install/setup.bash ]; then
  source /opt/turtlebot3_ws/install/setup.bash
fi

# Hand off to whatever the user specified
exec "$@"
