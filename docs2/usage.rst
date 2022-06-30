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

    ros2 run crazyswarm2_examples hello_world --sim


Physical Experiments
--------------------

ROS2 terminal
~~~~~~~~~~~~~

.. code-block:: bash

    ros2 param set crazyswarm2_server cf1/params/commander/enHighLevel 1
    ros2 param set crazyswarm2_server cf3/params/stabilizer/estimator 2
    ros2 service call cf1/takeoff crazyswarm2_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 2}}"
    ros2 service call cf1/land crazyswarm2_interfaces/srv/Land "{height: 0.0, duration: {sec: 2}}"

Teleoperation
~~~~~~~~~~~~~

We currently assume an XBox controller (the button mapping can be changed in teleop.yaml). It is possible to fly in different modes, including attitude-control and position-control (in which case any localization system can assist.)

.. code-block:: bash

    ros2 launch crazyswarm2 launch.py


Python scripts
~~~~~~~~~~~~~~

In the first terminal, launch

.. code-block:: bash

    ros2 launch crazyswarm2 launch.py

In the second terminal

.. code-block:: bash

    ros2 run crazyswarm2_examples hello_world