[![ROS2](https://github.com/IMRCLab/crazyswarm2/actions/workflows/ci-ros2.yml/badge.svg)](https://github.com/IMRCLab/crazyswarm2/actions/workflows/ci-ros2.yml)

# Crazyswarm2
A ROS2-based stack for swarms of Bitcraze Crazyflie multirotor robots.

## Current Status

* CI for Galactic (Ubuntu only)
* crazyswarm2 package
  * Former crazyflie_tools, e.g., `ros2 run crazyswarm2 console`
  * Former crazyswarm_teleop
* crazyswarm2_interfaces package
  * All msg/srv files similar to before (updated to follow the new style guide)
* crazyswarm2_server package
  * Firmware parameters (mapping to parameter server with callback on updates)
* Standalone tracking package, see https://github.com/IMRCLab/motion_capture_tracking/tree/ros2

## Missing

* Former crazyswarm_server
  * Most features are still missing
* Simulation
* Scripting layer

## Building and Running

```
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/IMRCLab/crazyswarm2 --recursive
cd ../
colcon build --packages-select crazyswarm2 crazyswarm2_interfaces py_crazyswarm2 crayzswarm2_examples
. install/local_setup.zsh (OR . install/local_setup.bash)
ros2 run crazyswarm2 console
```

## Notes

### Basic High-Level Flight

```
ros2 param set cf1/params/commander/enHighLevel 1
ros2 service call cf1/takeoff crazyswarm2_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 2}}"
ros2 service call cf1/land crazyswarm2_interfaces/srv/Land "{height: 0.0, duration: {sec: 2}}"
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

