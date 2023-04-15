.. _howtos:

How To
======

This page shows short how to's for the advanced usage.


Tracking Non-Robotic Objects
----------------------------

In some cases it can be useful to provide the position or pose of rigid bodies in the motion capture space to the robots.
For example, consider a collision avoidance algorithms implemented on-board the firmware that requires to know
the position of an obstacle. In that case, a "virtual" robot can be defined in crazyflies.yaml

.. code-block:: yaml

    robots:
        obstacle:
            enabled: true
            initial_position: [0, -0.5, 0]
            type: marker  # see robot_types
            id: 255

    robot_types:
        # Just a marker to track, not a robot to connect to
        marker:
            connection: none
            motion_capture:
                enabled: true
                marker: default_single_marker
                dynamics: default


Here, the position of the obstacle will be tracked using the motion_capture_tracking ROS package. The resulting pose will be
available via TF (named "obstacle") and send to the firmware of the actual robots with id 255.

Debugging
---------

If there is a crash (e.g., segmentation fault) in the crazyflie_server (C++ backend), you can find the stacktrace by using gdb.
First, compile your code in debug mode, then run the launch file with the debug flag, which will open an xterm window.
If you don't have xterm installed, you can do so using `sudo apt install xterm`.

.. code-block:: bash

    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
    ros2 launch crazyflie launch.py debug:=True

