.. _faq:

Frequently Asked Questions
==========================

How can I get help?
-------------------

Please start a `discussion <https://github.com/IMRCLab/crazyswarm2/discussions>`_ for

- Getting Crazyswarm2 to work with your hardware setup.
- Advice on how to use it to achieve your goals.
- Rough ideas for a new feature.


How can I report an issue?
--------------------------

If you strongly believe that you found a bug that requires changes in the code, feel free to open an `issue <https://github.com/IMRCLab/crazyswarm2/issues>`_.
Provide as much details as possible to reproduce the problem, your observation, as well as the desired outcome.

If you are not sure if a code change is required, please start a `discussion <https://github.com/IMRCLab/crazyswarm2/discussions>`_ first.


How do Crazyswarm2 and Crazyswarm differ?
-----------------------------------------

Crazyswarm2 was forked from Crazyswarm. However, there is also heavy re-design of core design choices.

- **Motion capture integration.**
  In Crazyswarm1, the motion capture integration is part of the crazyswarm_server, due to limitations in the ROS1 pub/sub system.
  In contrast, Crazyswarm2 now follows a better ROS-style design, where our motion capture abstraction layer and custom
  frame-by-frame tracking is available as a `separate ROS2 package <https://github.com/IMRCLab/motion_capture_tracking>`_.
  In addition, Crazyswarm2 is designed to support other localization methods (lighthouse, or on-board only localization) from the start.

- **Communication backend.**
  In Crazyswarm2, we rely on `crazyflie-link-cpp <https://github.com/bitcraze/crazyflie-link-cpp>`_ at the lowest layer, unlike a custom link implementation in Crazyswarm1.
  This new link uses priority queues that allows new features like uploading trajectories during the flight. Moreover, the new link should improve the overall (communication) robustness.
  There is also experimental support for a `crazyflie-lib-python <https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/>`_ backend as well. 

- **Support for Common UAV tasks.**
  Crazyswarm1 was designed to operate a swarm and did not include common features for single-robot operation, such as teleoperation. Crazyswarm2 replaces both Crazyswarm1 and crazyflie_ros, thus supporting arbitrary URIs, teleoperation etc.

- **Simulation.**
  In Crazyswarm1, a simple visualization of the setpoints for high-level Python scripts is supported. There is no support for simulation of ROS code that does not use the high-level Python scripts and no support for physics-based simulation.
  In contrast, Crazyswarm2 implements the simulation as an alternative backend. This will support multiple physics/visualization backends (optionally with physics and aerodynamic interaction).

- **Support of Distributed Swarm Monitoring (Planned).**
  In Crazyswarm1, a common swarm monitoring tool is the chooser.py (to enable/disable CFs, check the battery voltage etc.). However, this tool was not functioning while the swarm is operational.
  In contrast, Crazyswarm2 will allow common swarm monitoring tasks without restarting ROS nodes or launching additional tools. 


How is Crazyswarm2 different from Bitcraze's cflib?
---------------------------------------------------

Both can be used to control several Crazyflies from a Python script.
Here are some differences:

- **Motion capture integration.**
  Crazyswarm2 supports common motion capture systems using the ROS2 package `motion_capture_tracking <https://github.com/IMRCLab/motion_capture_tracking/tree/ros2>`_.
  The Bitcraze API can *send* position measurements to the Crazyflie,
  but does not know how to *get* position measurements from mocap hardware.
  Moreover, the use of motion_capture_tracking allows to use identical or single motion capture markers.
- **Broadcasts.**
  Crazyswarm2 uses broadcast communication whenever possible to require fewer radios per Crazyflie. In contrast, the official SDK uses unicast communication instead.
- **Simulation.**
  Crazyswarm2 has a simulation mode with 3D graphics,
  which makes it easy to validate complex scripts before running them on real hardware.
- **Python firmware bindings.**
  Crazyswarm2's simulator is built upon automatically generated Python bindings
  for certain modules in the Crazyflie firmware.
  The binding system can be helpful when developing new firmware modules,
  especially when they are mathematically complex and hard to debug.
- **ROS2 foundation.**
  The Crazyswarm2 server program is a ROS2 node.
  Our Python API is a thin wrapper around the ROS2 interface.