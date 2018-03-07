Python API
==========

You can use the Python API like this::

    from pycrazyswarm import *
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

The ``swarm`` object gives you access to a time helper (that works in both real execution and simulation) and the CrazyflieServer (``allcfs``).

The CrazyflieServer object allows you to
  - Send broadcasts to a group of Crazyflies
  - Access individual Crazyflies

Broadcasts
----------

- ``allcfs.emergency(self)``
    Cut power to motors of all CFs
- ``allcfs.takeoff(self, targetHeight, duration, group = 0)``
    Take-off of given Crazyflie group to specified hight within the specified duration.
- ``allcfs.land(self, targetHeight, duration, group = 0)``
    Land given Crazyflie group to specified hight within the specified duration.
- ``allcfs.startTrajectory(self, group = 0)``
    The given Crazyflie group starts executing their uploaded trajectory.
- ``allcfs.startTrajectoryReversed(self, group = 0)``
    The given Crazyflie group starts executing their uploaded trajectory in reverse (i.e., from end to beginning).
- ``allcfs.startEllipse(self, group = 0)``:
- ``allcfs.startCannedTrajectory(self, trajectory, timescale, group = 0)``:
- ``allcfs.goHome(self, group = 0)``:
- ``allcfs.setParam(self, name, value, group = 0)``:

Access Crazyflies
-----------------

- ``cf = allcfs.crazyflies[idx]``
  Array that contains all Crazyflies
- ``cf = allcfs.crazyfliesById["42"]``
  Dictionary that contains crazyflies by their ids

Individual Crazyflie
--------------------

- ``cf.id``
   ID of the crazyflie
- ``cf.initialPosition``
   Initial position (as defined in ``crazyflies.yaml``)
- ``cf.uploadTrajectory(self, firmware_trajectory)``
- ``cf.setEllipse(self, center, major, minor, period)``
- ``cf.takeoff(self, targetHeight, duration)``
- ``cf.land(self, targetHeight, duration)``
- ``cf.hover(self, goal, yaw, duration)``
    Move to the specified goal (position) and yaw angle in the specified time.
- ``cf.setGroup(self, group)``
    Assign this Crazyflies to be part of the given group (all CFs are part of group 0)
- ``cf.position(self)``
    Returns the current position of the Crazyflie
- ``cf.getParam(self, name)``
- ``cf.setParam(self, name, value)``

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
        cf.hover(pos, 0, 1.0)

    # Wait 5 seconds
    timeHelper.sleep(5)

    # Land
    allcfs.land(targetHeight=0.02, duration=2.0)
    timeHelper.sleep(2.0)

