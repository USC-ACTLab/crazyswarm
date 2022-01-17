[![ROS2](https://github.com/IMRCLab/crazyswarm2/actions/workflows/ci-ros2.yml/badge.svg)](https://github.com/IMRCLab/crazyswarm2/actions/workflows/ci-ros2.yml)

# Crazyswarm2
A ROS2-based stack for swarms of Bitcraze Crazyflie multirotor robots.

## Current Status

* CI for Galactic (Ubuntu only)
* crazyswarm2 package
  * Former crazyflie_tools, e.g., `ros2 run crazyswarm2 console`
  * Former crazyswarm_teleop
  * Former chooser.py
  * crazyswarm_server
    * Firmware parameters (mapping to parameter server with callback on updates)
    * High-level Takeoff/Landing/GoTo/StartTrajectory (per CF, and broadcasts)
    * Broadcasting motion capture information (position only)
* crazyswarm2_interfaces package
  * All msg/srv files similar to before (updated to follow the new style guide)

* py_crazyswarm2
  * Former Python API (not all functions are ported)
* crazyswarm2_examples
  * Former example scripts (currently: only hello_world, nice_hover, figure8)
* Standalone tracking package, see https://github.com/IMRCLab/motion_capture_tracking/tree/ros2

## Missing

* Former crazyswarm_server
  * Data logging
  * broadcast motion capture full pose information
* Scripting layer
  * limited feature set
* Chooser.py
  * No support for flashing firmware
  * Might only work with `--symlink-install`
  * TODO: should be replaced by a tool that connects to the server instead

## Building and Running

```
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/IMRCLab/crazyswarm2 --recursive
git clone --branch ros2 --recursive https://github.com/IMRCLab/motion_capture_tracking.git
cd ../
colcon build --symlink-install
```
Note: symlink-install allows you to edit Python and config files without running `colcon build` every time.

In a separate terminal:
```
. install/local_setup.zsh (OR . install/local_setup.bash)
ros2 run crazyswarm2 console
```

### Simulator with Software In The Loop

```
cd py_crazyswarm2/py_crazyswarm2/cfsim
make
```

To run, use the `--sim` flag, e.g., 

```
ros2 run crazyswarm2_examples hello_world
```

## Notes

### Basic High-Level Flight

```
ros2 param set crazyswarm2_server cf1/params/commander/enHighLevel 1
ros2 param set crazyswarm2_server cf3/params/stabilizer/estimator 2
ros2 service call cf1/takeoff crazyswarm2_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 2}}"
ros2 service call cf1/land crazyswarm2_interfaces/srv/Land "{height: 0.0, duration: {sec: 2}}"
```

### Chooser.py

```
ros2 run crazyswarm2 chooser.py
```

### crazyswarm2_examples

```
ros2 run crazyswarm2_examples hello_world
```

## OLD - TO BE UPDATED

![Crazyswarm ROS CI](https://github.com/USC-ACTLab/crazyswarm/workflows/Crazyswarm%20ROS%20CI/badge.svg)
![Sim-Only Conda CI](https://github.com/USC-ACTLab/crazyswarm/workflows/Sim-Only%20Conda%20CI/badge.svg)
[![Documentation Status](https://readthedocs.org/projects/crazyswarm/badge/?version=latest)](https://crazyswarm.readthedocs.io/en/latest/?badge=latest)

The documentation is available here: http://crazyswarm.readthedocs.io/en/latest/.

## Troubleshooting
Please start a [Discussion](https://github.com/USC-ACTLab/crazyswarm/discussions) for...

- Getting Crazyswarm to work with your hardware setup.
- Advice on how to use the [Crazyswarm Python API](https://crazyswarm.readthedocs.io/en/latest/api.html) to achieve your goals.
- Rough ideas for a new feature.

Please open an [Issue](https://github.com/USC-ACTLab/crazyswarm/issues) if you believe that fixing your problem will involve a **change in the Crazyswarm source code**, rather than your own configuration files. For example...

- Bug reports.
- New feature proposals with details.

