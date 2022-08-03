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

- **Support for Common UAV tasks.**
  Crazyswarm1 was designed to operate a swarm and did not include common features for single-robot operation, such as teleoperation. Crazyswarm2 replaces both Crazyswarm1 and crazyflie_ros, thus supporting arbitrary URIs, teleoperation etc.

- **Simulation (Planned).**
  In Crazyswarm1, a simple visualization of the setpoints for high-level Python scripts is supported. There is no support for simulation of ROS code that does not use the high-level Python scripts and no support for physics-based simulation.
  In contrast, Crazyswarm2 will support multiple simulation backends (optionally with physics and aerodynamic interaction) that work independent of the choice of API (by mimicking the API of the crazyflie_server).

- **Support of Distributed Swarm Monitoring (Planned).**
  In Crazyswarm1, a common swarm monitoring tool is the chooser.py (to enable/disable CFs, check the battery voltage etc.). However, this tool was not functioning while the swarm is operational.
  In contrast, Crazyswarm2 will allow common swarm monitoring tasks without restarting ROS nodes or launching additional tools. 

