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
- ``allcfs.takeoff(self, targetHeight, duration, groupMask = 0)``
    Take-off of given Crazyflie group to specified hight within the specified duration.
- ``allcfs.land(self, targetHeight, duration, groupMask = 0)``
    Land given Crazyflie group to specified hight within the specified duration.
- ``allcfs.stop(self, groupMask = 0)``
    Stops (i.e., turns off motors) for given Crazyflie group.
- ``goTo(self, goal, yaw, duration, groupMask = 0)``
    Moves each Crazyflie relative to its current position/yaw by the specified goal/yaw offset and reaches that location after the specified duration.
- ``startTrajectory(self, trajectoryId, timescale = 1.0, reverse = False, relative = True, groupMask = 0)``
    Starts executing the specified trajectory. Trajectory can be scaled in time (larger number = slower), or executed in reverse.

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
- ``cf.setGroupMask(self, groupMask)``
    Assign this Crazyflies to be part of the given groups (all CFs are part of group 0)
- ``cf.takeoff(self, targetHeight, duration, groupMask = 0)``
- ``cf.land(self, targetHeight, duration, groupMask = 0)``
- ``cf.stop(self, groupMask = 0)``
- ``cf.goTo(self, goal, yaw, duration, relative = False, groupMask = 0)``
    Move to the specified goal (position) and yaw angle in the specified time.
- ``cf.uploadTrajectory(self, trajectoryId, pieceOffset, trajectory)``
- ``cf.startTrajectory(self, trajectoryId, timescale = 1.0, reverse = False, relative = True, groupMask = 0)``
- ``cf.position(self)``
    Returns the current position of the Crazyflie
- ``cf.getParam(self, name)``
- ``cf.setParam(self, name, value)``
- ``cf.setParams(self, params)``

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

