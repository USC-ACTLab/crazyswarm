.. _tutorial_hover:

Hovering (hello, world)
------------------------

After completing :ref:`Installation` and :ref:`Configuration`,
you are ready to test your setup for the first time!

First, run the test script in simulation mode to make sure your Python interpreter is set up correctly::

    python hello_world.py --sim

In the 3D visualization, you should see a Crazyflie take off, hover for a few seconds, and then land.

If the script runs in simulation, you can move on to real hardware.
First, start the ``crazyswarm_server``::

    source ros_ws/devel/setup.bash
    roslaunch crazyswarm hover_swarm.launch

It should only take a few seconds to connect to the CFs.
If you have the LED ring extension installed, you can see the connectivity by the color (green=good connectivity; red=bad connectivity).
Furthermore, ``rviz`` will show the estimated pose of all CFs.
If there is an error, such as a faulty configuration or a turned-off Crazyflie, an error message will be shown and the application exits.
If there is a problem in the communication between the motion capture system and the Crazyswarm server, the application will not exit but the positions of the Crazyflies will not appear in rviz.

If ``crazyswarm_server`` is running correctly and you see CF pose(s) in ``rviz``,
open a new terminal (remember to ``source devel/setup.bash``) and run the test script::

    python hello_world.py

You should see the same behavior in real life.
If you have more than one Crazyflie in your ``crazyflies.yaml``,
the script will select one arbitrarily.
