.. _usage:

Usage
=====

.. warning::
    Do not forget to source your ROS2 workspace in each terminal you would like to use it

    .. code-block:: bash

        . install/local_setup.bash


Configuration
-------------

All configuration files are in crazyswarm2/config.

Simulation
----------

High-level Python scripts can be visualized before execution. The initial position and number of robots is taken from the crazyflies.yaml configuration file.
The simulation uses the firmware code as software-in-the-loop, but (currently) does not include any dynamics.

Example:

.. code-block:: bash

    ros2 run crazyflie_examples hello_world --sim


Physical Experiments
--------------------

ROS2 terminal
~~~~~~~~~~~~~

.. code-block:: bash

    ros2 run crazyflie reboot --uri radio://0/80/2M/E7E7E7E706 --mode sysoff

    ros2 param set crazyflie cf1/params/commander/enHighLevel 1
    ros2 param set crazyflie cf1/params/stabilizer/estimator 2
    ros2 service call cf1/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 2}}"
    ros2 service call cf1/land crazyflie_interfaces/srv/Land "{height: 0.0, duration: {sec: 2}}"

Teleoperation
~~~~~~~~~~~~~

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

RVIZ2 Pose Vizualization (Only CFlib backend)
~~~~~~~~~~~~~~

In crazyflie.yaml, make sure that this following is added or uncommented

.. code-block:: bash
    all:
    ...
    firmware_logging:
        enabled: true
        default_topics:
        pose:
            frequency: 10 # Hz

In the first terminal, launch

.. code-block:: bash

    ros2 launch crazyflie launch.py backend:=cflib

In the second terminal

.. code-block:: bash

    rviz2

Then set 'fixed frame' to 'world' and add the TF plugin. Then in 'TF', check  the 'show names' checkbox.
The crazyflie names should appear with their estimated position.