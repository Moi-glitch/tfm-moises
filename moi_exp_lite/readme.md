# moi_exp_lite

This repository provides the `moi_exp_lite` ROS 2 package. It contains a C++
implementation of the explore-lite algorithm along with Python nodes for
traffic-light detection and a controller used during exploration. The package
is mainly targeted at the TurtleBot3 platform.

## Project overview

The main workflow is:

1. **Exploration** – the `explore_node` (C++) searches for frontiers in the
   navigation costmap and publishes navigation goals.
2. **Traffic-light detection** – `detect_object` (Python) processes the
   camera feed and publishes detections on `/color_detection`.
3. **Explore controller** – `explore_controller` (Python) listens for detections,
   transforms them into the map frame and publishes RViz markers. Detected
   objects are stored in `detected_objects.yaml`.

Launch files under `launch/` provide scripts to start these nodes individually
or together. `moi_exploration.launch.py` runs the full calibration and
exploration sequence.

## Building the package

Before building, install all system and ROS dependencies declared in
`package.xml`.  A helper script is provided which runs `rosdep` for you:

```bash
./rosdep_install.sh
```

This command updates the rosdep database and installs any missing packages
required by `moi_exp_lite`.  The script expects that your ROS distribution is
already installed and that you have permission to use `sudo` for package
installation.

Build with `colcon` from the workspace root:

```bash
colcon build --symlink-install
```


## Running the nodes

Two standalone launch files are provided:

* `explore.launch.py` – starts the `explore_node` from this package. Optional
  arguments allow setting a parameter file and toggling use of simulated time.
  Example:
  ```bash
  ros2 launch moi_exp_lite explore.launch.py
  ```
* `detect_object.launch.py` – launches the colour detector. The argument
  `calibration_mode` can be set to `True` or `False`:
  ```bash
  ros2 launch moi_exp_lite detect_object.launch.py calibration_mode:=False
  ```

The `moi_exploration.launch.py` script chains calibration, detection and
exploration together.

## Configuration notes

The package is self-contained and does not rely on absolute paths.  All
parameter files, maps and behavior-tree XML are installed under the package
share directory.  Launch files resolve these resources at runtime so the
package can be built in any workspace.

The Behavior Tree visualiser (`groot2`) can be overridden via the
`GROOT2_EXECUTABLE` environment variable.  If the variable is not set the
default `groot2` executable is used.

