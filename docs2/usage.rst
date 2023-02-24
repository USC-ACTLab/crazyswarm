.. _usage:

Usage
=====

.. warning::
    Do not forget to source your ROS 2 workspace in each terminal you would like to use it

    .. code-block:: bash

        . install/local_setup.bash


.. warning::
    If you work in a shared network (lab, classroom) or similar, you might want to avoid 
    controlling other robots. This is in particular true for simulation. In this case, 
    you can use

    .. code-block:: bash

        export ROS_LOCALHOST_ONLY=1


Configuration
-------------

All configuration files are in crazyflie/config. 

* crazyflies.yaml : setting up everything related to the robots.
* server.yaml : setting up everything related to the server.
* motion_capture.yaml : configs for the motion capture package.
* teleop.yaml : configs for remote controls.

crazyflies.yaml
~~~~~~~~~~~~~~~

Each crazyflie should have an unique URI which can `be changed in Bitcraze's CFclient <https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/#firmware-configuration/>`_.
They can also be enabled in case you don't want the server to connect with it. 

.. code-block:: yaml

    robots:
        cf231:
            enabled: true
            uri: radio://0/80/2M/E7E7E7E7E7
            initial_position: [0, 0, 0]
            type: cf21  # see robot_types

        cf5:
            enabled: false
            uri: radio://0/80/2M/E7E7E7E705
            initial_position: [0, -0.5, 0]
            type: cf21  # see robot_types

The yaml file also contains different robot_types, to indicate differences between each platform:

.. code-block:: yaml

    robot_types:
        cf21:
            motion_capture:
                enabled: true
                # only if enabled; see motion_capture.yaml
                marker: default_single_marker
                dynamics: default
            big_quad: false
            battery:
                voltage_warning: 3.8  # V
                voltage_critical: 3.7 # V

        cf21_mocap_deck:
            motion_capture:
                enabled: true
                # only if enabled; see motion_capture.yaml
                marker: mocap_deck
                dynamics: default
            big_quad: false
            battery:
                voltage_warning: 3.8  # V
                voltage_critical: 3.7 # V

The yaml file also contain an 'all' field, in case you have parameters or logging that you want enabled for all the connected crazyflies.


.. code-block:: yaml

    all:
        firmware_logging:
            enabled: false
            default_topics:
                pose:
                frequency: 10 # Hz
            #custom_topics:
            #  topic_name1:
            #    frequency: 10 # Hz
            #    vars: ["stateEstimateZ.x", "stateEstimateZ.y", "stateEstimateZ.z", "pm.vbat"]
            #  topic_name2:
            #    frequency: 1 # Hz
            #    vars: ["stabilizer.roll", "stabilizer.pitch", "stabilizer.yaw"]
        firmware_params:
            commander:
                enHighLevel: 1
            stabilizer:
                estimator: 2 # 1: complementary, 2: kalman
                controller: 2 # 1: PID, 2: mellinger

The above also contains an example of the firmware_logging field, where default topics can be enabled or custom topics based on the `existing log toc of the crazyflie <https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs//>`_. 
Moreover, it also contains the firmware_params field, where parameters can be set at startup. 
Also see the `parameter list of the crazyflie <https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/params//>`_ for that. 


Mind that you can also place the firmware_params and firmware_logging fields per crazyflie in 'robots'  or the 'robot_types' field.
The server node will upon initialization, first look at the params/logs from the individual crazyflie's settings, then the robot_types, and then anything in 'all' which has lowest priority.  


Simulation
----------

Any usage of the ROS API, including high-level Python scripts, can be visualized before execution. The initial position and number of robots is taken from the crazyflies.yaml configuration file.
The simulation uses the firmware code as software-in-the-loop, and can optionally include the robot dynamics.
The configuration of the simulation (physics simulator, controller, etc.) can be changed in server.yaml.

Example:

.. code-block:: bash

    ros2 launch crazyflie launch.py backend:=sim
    ros2 run crazyflie_examples hello_world --ros-args -p use_sim_time:=True

Physical Experiments
--------------------

ROS 2 terminal
~~~~~~~~~~~~~~

The following shows a simple take off and land example without any launch files or yaml files

.. code-block:: bash

    ros2 run crazyflie reboot --uri radio://0/80/2M/E7E7E7E706 --mode sysoff
    ros2 param set crazyflie_server cf1.params.commander.enHighLevel 1
    ros2 param set crazyflie_server cf1.params.stabilizer.estimator 2
    ros2 service call cf1/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 2}}"
    ros2 service call cf1/land crazyflie_interfaces/srv/Land "{height: 0.0, duration: {sec: 2}}"

Enabling Logblocks at runtime
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. warning::
    Runtime logblock enabling is currently only supported in the CFLIB backend of the server.

Above we explained how to enable log blocks at startup, but what if you would like to enable or disable logging blocks in runtime?
This section will show how to do that by using services

In one terminal run

.. code-block:: bash

    ros2 launch crazyflie launch.py backend:=cflib

In another terminal after sourcing the right setup.bash files, run:

.. code-block:: bash

    ros2 service call /cf2/add_logging crazyflie_interfaces/srv/AddLogging "{topic_name: 'topic_test', frequency: 10, vars: ['stateEstimate.x','stateEstimate.y','stateEstimate.z']}"
    ros2 service call /cf2/add_logging crazyflie_interfaces/srv/AddLogging "{topic_name: 'pose', frequency: 10}

With ROS 2's rqt you can look at the topics, or with 'ROS 2 topics echo /cf2/pose'

To close the logblocks again, run:

.. code-block:: bash

    ros2 service call /cf2/remove_logging crazyflie_interfaces/srv/RemoveLogging "{topic_name: 'topic_test'}"
    ros2 service call /cf2/remove_logging crazyflie_interfaces/srv/RemoveLogging "{topic_name: 'pose'}"


Teleoperation controller
~~~~~~~~~~~~~~~~~~~~~~~~

We currently assume an XBox controller (the button mapping can be changed in teleop.yaml). It is possible to fly in different modes, including attitude-control and position-control (in which case any localization system can assist.)

.. code-block:: bash

    ros2 launch crazyflie launch.py


Python scripts
~~~~~~~~~~~~~~

In the first terminal, launch

.. code-block:: bash

    ros2 launch crazyflie launch.py

In the second terminal

.. code-block:: bash

    ros2 run crazyflie_examples hello_world
