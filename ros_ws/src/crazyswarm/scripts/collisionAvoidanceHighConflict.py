"""Worst-case test of Buffered Voronoi Cell collision avoidance algorithm.

Computes the center of all initial positions, and tells each robot to go straight across the center. Many are forced to wait. This stresses two components of the collision avoidance algorithm:

- Dealing with full "deadlocks" that require a sidestep maneuver
- Handling situations when the goals are far away from the current positions.

Also will cause high-gain controllers without proper windup handling to fail.
"""

from __future__ import print_function
import argparse

import numpy as np

import pycrazyswarm


Z = 1.0  # Takeoff altitude.
duration = 6.0  # Duration of all goTos.


def positionGoTo(timeHelper, cfs, goals, kp=1.0, velMax=0.5):
    while True:
        positions = np.row_stack([cf.position() for cf in cfs])
        errors = positions - goals

        # Check stopping criterion.
        distances = np.linalg.norm(errors, axis=1)
        assert len(distances) == len(cfs)
        if np.all(distances < 0.1):
            break

        # Command direct to goal.
        # Rely on BVCA algorithm and controller to not be too aggressive.
        for cf, goal in zip(cfs, goals):
            cf.cmdPosition(goal, yaw=0)

        timeHelper.sleepForRate(30)


def velocityGoTo(timeHelper, cfs, goals, kp=2.0, velMax=0.5):
    while True:
        positions = np.row_stack([cf.position() for cf in cfs])
        errors = positions - goals

        # Check stopping criterion.
        distances = np.linalg.norm(errors, axis=1)
        assert len(distances) == len(cfs)
        if np.all(distances < 0.1):
            break

        # Compute P-only linear position control law.
        # Rely on BVCA algorithm and controller to not be too aggressive.
        velocities = -kp * errors
        for cf, vel in zip(cfs, velocities):
            cf.cmdVelocityWorld(vel, yawRate=0)

        timeHelper.sleepForRate(30)


def main():
    # Crazyswarm's inner parser must add help to get all params.
    parser = argparse.ArgumentParser(add_help=False)
    group = parser.add_argument_group("Collision avoidance", "")
    group.add_argument(
        "--mode",
        help="Control mode to use.",
        choices=["goto", "velocity", "position"],
        default="goto",
    )
    args, unknown = parser.parse_known_args()

    swarm = pycrazyswarm.Crazyswarm(parent_parser=parser)
    timeHelper = swarm.timeHelper
    cfs = swarm.allcfs.crazyflies

    xy_radius = 0.3
    radii = xy_radius * np.array([1.0, 1.0, 3.0])

    for i, cf in enumerate(cfs):
        others = cfs[:i] + cfs[(i+1):]
        cf.enableCollisionAvoidance(others, radii)

    # Everyone will go straight across the center, causing many conflicts.
    initialPositions = np.row_stack([cf.initialPosition for cf in cfs])
    initialPositions[:,2] = Z
    center = np.mean(initialPositions, axis=0)
    goals = center + (center - initialPositions)

    swarm.allcfs.takeoff(targetHeight=Z, duration=Z+1.0)
    timeHelper.sleep(Z + 2.0)

    if args.mode == "goto":
        for goal, cf in zip(goals, cfs):
            cf.goTo(goal, yaw=0.0, duration=duration)
        timeHelper.sleep(duration + 1.0)
        for cf in cfs:
            cf.goTo(cf.initialPosition + [0.0, 0.0, Z], yaw=0.0, duration=duration)
        timeHelper.sleep(duration + 1.0)

    elif args.mode == "velocity":
        velocityGoTo(timeHelper, cfs, goals)
        velocityGoTo(timeHelper, cfs, initialPositions)
        for cf in cfs:
            cf.notifySetpointsStop()

    elif args.mode == "position":
        positionGoTo(timeHelper, cfs, goals)
        positionGoTo(timeHelper, cfs, initialPositions)
        for cf in cfs:
            cf.notifySetpointsStop()

    else:
        raise ValueError("--mode {} not understood.".format(args.mode))

    for cf in cfs:
        cf.disableCollisionAvoidance()

    swarm.allcfs.land(targetHeight=0.04, duration=Z+1.0)
    timeHelper.sleep(Z + 2.0)


if __name__ == "__main__":
    main()
