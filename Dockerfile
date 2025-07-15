FROM ros:humble-ros-base

# Environment noninteractive
ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger

# Install colcon and other build tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool build-essential git \
    ros-humble-turtlebot3* \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /opt/ws
RUN mkdir src

# Copy package sources
COPY . src/

# Install dependencies
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Default command: source environment and start bash
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && exec bash"]
