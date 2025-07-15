# Dockerfile for TurtleBot3 Simulation Workspace

# 1) Base image
ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop AS base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# 2) Install OS & ROS deps (including TB3 sims + Nav2)
RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      libcanberra-gtk-module libcanberra-gtk3-module \
      fuse3 libfuse2 libqt5svg5-dev \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      build-essential \
      ros-${ROS_DISTRO}-turtlebot3-simulations \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-nav2-util \
      ros-${ROS_DISTRO}-nav2-msgs \
      ros-${ROS_DISTRO}-nav2-costmap-2d \
      ros-${ROS_DISTRO}-nav2-core \
      ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
      ros-${ROS_DISTRO}-slam-toolbox \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# 3) GUI extras (X11 apps)
RUN apt-get update && apt-get install -y --no-install-recommends \
      x11-apps \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# 4) Python libs you need
RUN pip3 install \
      numpy==1.21.6 \
      scipy==1.7.3 \
      matplotlib==3.5.1 \
      transforms3d==0.4.1

# 5) Create & populate your workspace
WORKDIR /opt/turtlebot3_ws
RUN mkdir src
COPY . src/

# 6) Bring in your package-specific rosdep script
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3_autorace.git src/turtlebot3_autorace
COPY moi_exp_lite/rosdep_install.sh /rosdep_install.sh
RUN chmod +x /rosdep_install.sh


# Remove ament_python from package.xml so rosdep won't choke on it
RUN sed -i '/<buildtool_depend>ament_python<\/buildtool_depend>/d' src/moi_exp_lite/package.xml \
 && sed -i '/<depend>ament_python<\/depend>/d'          src/moi_exp_lite/package.xml

# 7) Install all dependencies & build
# 7) Install all dependencies & build
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    # a) deps for moi_exp_lite
    cd src/moi_exp_lite && /rosdep_install.sh && cd ../.. && \
    # b) rest of workspace deps (skip problematic keys) + build
    rosdep update && \
    rosdep install --from-paths src \
                   --ignore-src \
                   -r \
                   --skip-keys ament_python \
                   --skip-keys turtlebot3-autorace-camera \
                   -y && \
    colcon build --symlink-install


# 8) Default TB3 model for simulation
ENV TURTLEBOT3_MODEL=burger_cam

# 9) Copy & normalize your entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN apt-get update && apt-get install -y --no-install-recommends dos2unix && \
    dos2unix /entrypoint.sh && \
    sed -i '1s/^\xEF\xBB\xBF//' /entrypoint.sh && \
    chmod +x /entrypoint.sh && \
    rm -rf /var/lib/apt/lists/*

# 10) Entrypoint + default command
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
