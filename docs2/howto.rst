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

Usage from the command line
---------------------------

The following shows a simple take off and land example

.. code-block:: bash

    [terminal1]$ ros2 run crazyflie reboot --uri radio://0/80/2M/E7E7E7E706 --mode sysoff
    [terminal2]$ ros2 launch crazyflie launch.py
    [terminal1]$ ros2 param set crazyflie_server cf1.params.commander.enHighLevel 1
    [terminal1]$ ros2 param set crazyflie_server cf1.params.stabilizer.estimator 2
    [terminal1]$ ros2 service call cf1/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 2}}"
    [terminal1]$ ros2 service call cf1/land crazyflie_interfaces/srv/Land "{height: 0.0, duration: {sec: 2}}"

Enabling Logblocks at runtime
-----------------------------

.. warning::
    Runtime logblock enabling is currently only supported in the CFLIB backend of the server.

In the usage we explained how to enable log blocks at startup, but what if you would like to enable or disable logging blocks in runtime?
This section will show how to do that by using services

In one terminal run

.. code-block:: bash

    ros2 launch crazyflie launch.py backend:=cflib

In another terminal after sourcing the right setup.bash files, run:

.. code-block:: bash

    ros2 service call /cf2/add_logging crazyflie_interfaces/srv/AddLogging "{topic_name: 'topic_test', frequency: 10, vars: ['stateEstimate.x','stateEstimate.y','stateEstimate.z']}"
    ros2 service call /cf2/add_logging crazyflie_interfaces/srv/AddLogging "{topic_name: 'pose', frequency: 10}"

With ROS 2's rqt you can look at the topics, or with 'ROS 2 topics echo /cf2/pose'

To close the logblocks again, run:

.. code-block:: bash

    ros2 service call /cf2/remove_logging crazyflie_interfaces/srv/RemoveLogging "{topic_name: 'topic_test'}"
    ros2 service call /cf2/remove_logging crazyflie_interfaces/srv/RemoveLogging "{topic_name: 'pose'}"

Run Tests Locally
-----------------

This requires some updated pip packages for testing, see https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html, otherwise the reported failures will be inconsistent with CI.

Then execute:

.. code-block:: bash

    colcon test --event-handlers=console_cohesion+ --return-code-on-test-failure --packages-select crazyflie_py

.. _Collision Avoidance:

Collision Avoidance
-------------------

The official firmware has support for collision avoidance using the Buffered Voronoi Cell algorithm.
It requires the use of a motion capture system (so that the positions of other drones are known) and can be enabled
in the `crazyflies.yaml`:

.. code-block:: yaml

    all:
        firmware_params:
            colAv:
                enable: 1

or inside a Python script via:

.. code-block:: python

    swarm = Crazyswarm()
    allcfs = swarm.allcfs
    allcfs.setParam("colAv.enable", 1)

Note that the algorithm might require tuning of its hyperparameters. Documention can be found at https://github.com/bitcraze/crazyflie-firmware/blob/dbb9df1137f11d4e7e3771c56d25a7137b5b69cc/src/modules/src/collision_avoidance.c#L348-L428.

Generate Trajectories
---------------------

Crazyswarm2 supports polynomial trajectories (8th order). These can be generated from waypoints, waypoint/time pairs, or optimization. Useful tools are available at https://github.com/whoenig/uav_trajectories, including scripts to visualize the resulting trajectories.

For the multi-robot case, there is no easy to-use library, yet. One can use collision avoidance (see :ref:`Collision Avoidance`) or preplan trajectories using https://github.com/IMRCLab/db-CBS or https://github.com/mjdebord/smoothener/tree/cylinders. 
