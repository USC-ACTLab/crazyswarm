Changelog
=========

October 4th, 2019
----------------

1. We switched to the official firmware. Our own EKF is not used anymore, instead we contributed a full pose update in the official EKF.
  a. We now support single-marker tracking, where only the position and not the pose is tracked by an external system. Attitude is estimated on-board.
  b. The EKF is pre-tuned to work with motion capture systems and the Mellinger controller.
2. There can be now more than 255 parameters in the firmware. Parameters can be specified by Crazyflie (and not only per type as before). Parameters can be updated using broadcasts mid-flight (e.g., to switch the controller mid-flight).
3. We now have full support for OptiTrack, including for using Motive 2.x. Both object tracking by Motive and custom object tracking are supported.

April 22nd, 2018
----------------

1. More features have been merged into the official firmware. The high-level execution (takeoff, landing, trajectory execution) is now part of the official firmware. Custom firmware changes still include:

  a. Kalman filter that support full-pose update
  b. Improved handling after crashes (particularly important for big quads)
  c. Custom LED-ring effect

2. The Crazyswarm now uses the latest official crazyflie_ros instead of its own fork. Crazyswarm specific code (e.g, crazyswarm_server) has been moved into ``ros_ws/src/crazyswarm/src``.

3. Getting the code into the official code base unfortunately required some API changes:

  a. Support for ellipsoids has been removed. If you need that feature please let us know via github issues.
  b. Support for canned trajectories has been removed.
  c. The new API allows to upload multiple trajectories and does not have a piece number limit (instead, it is limited by total memory size of all trajectories only).
  d. ``Hover`` has been renamed to ``GoTo``

March 2nd, 2018
---------------

#. Support for heterogeneous swarms, i.e., bigger quadrotors can be used with the CF as control board and the bigQuad deck. Types can be specified in ``ros_ws/src/crazyswarm/launch/crazyflieTypes.yaml``. The list of all CFs has now a type field in ``ros_ws/src/crazyswarm/launch/allCrazyflies.yaml`` (previously ``all49.yaml``). Each type can have its own marker configuration.

#. Firmware now re-based from the latest official firmware. Some parts are already in the official firmware (e.g., controller). The firmware now supports the use of our Kalman filter (KalmanUSC) or the official Kalman filter, although both have different feature sets. The prebuilt firmware uses our Kalman filter and comes with bigQuad-deck support enabled, i.e., it can be used for both standard Crazyflies and custom ones.

#. Removed support for avoid-target mode
