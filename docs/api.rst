.. _api:

Python API Reference
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

Note: rendering video output with the ``--video`` option requires an
installation of `ffmpeg <https://ffmpeg.org>`_ with the ``libx264`` encoder.
This is provided in the :ref:`anaconda` environment, but must be installed
manually otherwise.
