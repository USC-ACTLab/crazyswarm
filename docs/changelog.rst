Changelog
=========

March 2nd, 2018
---------------

#. Support for heterogeneous swarms, i.e., bigger quadrotors can be used with the CF as control board and the bigQuad deck. Types can be specified in ``ros_ws/src/crazyswarm/launch/crazyflieTypes.yaml``. The list of all CFs has now a type field in ``ros_ws/src/crazyswarm/launch/allCrazyflies.yaml`` (previously ``all49.yaml``). Each type can have its own marker configuration.

#. Firmware now re-based from the latest official firmware. Some parts are already in the official firmware (e.g., controller). The firmware now supports the use of our Kalman filter (KalmanUSC) or the official Kalman filter, although both have different feature sets. The prebuilt firmware uses our Kalman filter and comes with bigQuad-deck support enabled, i.e., it can be used for both standard Crazyflies and custom ones.
