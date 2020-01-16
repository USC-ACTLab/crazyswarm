Python Scripting API
====================

The module ``pycrazyswarm``, contained in ``/ros_ws/src/crazyswarm/scripts``,
is the main high-level interface to the Crazyswarm platform.

The goal of the Crazyswarm project is to reach a state where many diverse
multi-quadrotor research projects can be implemented without going any "deeper"
than writing a ``pycrazyswarm`` script and modifying configuration files.
New projects should try this approach first. If it becomes necessary to
modify Crazyswarm or its submodules, we encourage users to contribute those
changes back via Github pull request.

All classes in ``pycrazyswarm.crazyflie`` are mirrored by an identically named
class in ``pycrazyswarm.crazyflieSim``. The ``Sim`` version allows testing
``pycrazyswarm`` scripts in simulation with a 3D visualization before running
anything on real hardware. Since the APIs are identical, the documentation
only refers to the non-``Sim`` versions.


``Crazyflie`` class
-------------------
.. autoclass:: pycrazyswarm.crazyflie.Crazyflie
   :members:

``CrazyflieServer`` class
-------------------------
.. autoclass:: pycrazyswarm.crazyflie.CrazyflieServer
   :members:

``TimeHelper`` class
--------------------
.. autoclass:: pycrazyswarm.crazyflie.TimeHelper
   :members:


Switching between simulation and real hardware
----------------------------------------------
Correct ``pycrazyswarm`` scripts should be able to run in both simulation
and on real hardware without modification. Enable simulation and control the
simulation parameters with the command-line flags listed below.

.. argparse::
   :module: pycrazyswarm.crazyswarm
   :func: build_argparser
   :prog: python2 <my_crazyswarm_script.py>


Examples
--------

Nice Hover
^^^^^^^^^^

niceHover.py::

    import numpy as np
    from pycrazyswarm import *

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # takeoff
    allcfs.takeoff(targetHeight=1.0, duration=2.0)

    # move to the initially assigned positions facing forward
    timeHelper.sleep(2.0)
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
        cf.goTo(pos, 0, 1.0)

    # Wait 5 seconds
    timeHelper.sleep(5)

    # Land
    allcfs.land(targetHeight=0.02, duration=2.0)
    timeHelper.sleep(2.0)
