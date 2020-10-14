"""Worst-case test of Buffered Voronoi Cell collision avoidance algorithm.

Computes the center of all initial positions, and tells each robot to go straight across the center. Many are forced to wait. This stresses two components of the collision avoidance algorithm:

- Dealing with full "deadlocks" that require a sidestep maneuver
- Handling situations when the goals are far away from the current positions.

Also will cause high-gain controllers without proper windup handling to fail.
"""

from __future__ import print_function

import numpy as np

import pycrazyswarm


Z = 1.0  # Takeoff altitude.
duration = 6.0  # Duration of all goTos.


def main():

    swarm = pycrazyswarm.Crazyswarm()
    timeHelper = swarm.timeHelper
    cfs = swarm.allcfs.crazyflies

    xy_radius = 0.3
    radii = xy_radius * np.array([1.0, 1.0, 3.0])

    for i, cf in enumerate(cfs):
        others = cfs[:i] + cfs[(i+1):]
        cf.enableCollisionAvoidance(others, radii)

    # Everyone will go straight across the center, causing many conflicts.
    initialPositions = np.row_stack([cf.initialPosition for cf in cfs])
    center = np.mean(initialPositions, axis=0)
    goals = center + (center - initialPositions)
    goals[:,2] = Z

    swarm.allcfs.takeoff(targetHeight=Z, duration=Z+1.0)
    timeHelper.sleep(Z + 2.0)

    for goal, cf in zip(goals, cfs):
        cf.goTo(goal, yaw=0.0, duration=duration)
    timeHelper.sleep(duration + 1.0)

    for cf in cfs:
        cf.goTo(cf.initialPosition + [0.0, 0.0, Z], yaw=0.0, duration=duration)
    timeHelper.sleep(duration + 1.0)

    for cf in cfs:
        cf.disableCollisionAvoidance()

    swarm.allcfs.land(targetHeight=0.04, duration=Z+1.0)
    timeHelper.sleep(Z + 2.0)


if __name__ == "__main__":
    main()
