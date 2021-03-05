"""Demonstration of Buffered Voronoi Cell collision avoidance algorithm."""

from __future__ import print_function
import argparse
import time

import numpy as np
import scipy as sp
import scipy.spatial
import scipy.optimize

import pycrazyswarm
import pycrazyswarm.util as util
import pycrazyswarm.cfsim.cffirmware as firm


Z = 1.0  # Takeoff altitude.


def main():

    # Crazyswarm's inner parser must add help to get all params.
    parser = argparse.ArgumentParser(add_help=False)

    group = parser.add_argument_group("Collision avoidance", "")
    group.add_argument(
        "--noavoid",
        help="Disable collision avoidance.",
        action="store_true"
    )
    group.add_argument(
        "--assign",
        help=("Use optimal start-goal assignment instead of random assignment."),
        action="store_true"
    )
    group.add_argument(
        "--loops",
        type=int,
        default=1,
        help="Repeat the experiment this many times, without resetting start positions.",
    )
    args, unknown = parser.parse_known_args()

    if args.assign:
        duration = 3.0
    else:
        duration = 10.0

    # Construct the Crazyswarm objects.
    rows, cols = 3, 5
    N = rows * cols
    crazyflies_yaml = util.grid_yaml(rows, cols, spacing=0.5)
    swarm = pycrazyswarm.Crazyswarm(crazyflies_yaml=crazyflies_yaml, parent_parser=parser)
    timeHelper = swarm.timeHelper
    cfs = swarm.allcfs.crazyflies

    # Tell the visualizer to draw collision volume ellipsoids. Ellipsoids
    # change from green to red when a collision occurs. Make the radii slightly
    # smaller than the radii used for the collision avoidance algorithm as a
    # fudge factor.
    #
    # To show collisions on purpose, use --noavoid without --assign.
    xy_radius = 0.125
    radii = 1.0 * xy_radius * np.array([1.0, 1.0, 3.0])
    timeHelper.visualizer.showEllipsoids(0.95 * radii)

    swarm.allcfs.takeoff(targetHeight=Z, duration=Z+1.0)
    timeHelper.sleep(Z + 2.0)

    if not args.noavoid:
        for i, cf in enumerate(cfs):
            others = cfs[:i] + cfs[(i+1):]
            cf.enableCollisionAvoidance(others, radii)

    for _ in range(args.loops):
        # Goals are randomly positioned in the XY plane, with some random
        # perturbations in Z. If we didn't add the perturbations, we'd get purely
        # 2D movement that doesn't take advantage of the free space in the Z axis
        # to reduce conflicts.
        #
        # A more robust solution would be to add some random bias direction to each
        # robot's reference direction, so it will have a preferred altitude when
        # far away from the goal, giving Z separation even when the starts and
        # goals are all coplanar.
        #
        # Minimum goal distance of 5*radius ensures that robots are never trying to
        # squeeze too tightly in between two other robots.
        #
        goals = util.poisson_disk_sample(N, dim=2, mindist=5*xy_radius)
        goals_z = Z * np.ones(N) + 0.2 * np.random.uniform(-1.0, 1.0, size=N)
        goals = np.hstack([goals, goals_z[:,None]])

        starts = np.stack([cf.position() for cf in cfs])
        starts[:, 2] += Z

        all_pts = np.vstack([goals, starts])
        bbox_min = np.amin(all_pts, axis=0) - 4.0 * xy_radius
        bbox_max = np.amax(all_pts, axis=0) + 4.0 * xy_radius

        # Optimal assignment with sum of Euclidean distances objective guarantees
        # no collisions when following straight-line trajectories IF the robots
        # have no volume. If not, collisions may occur, but they are rare in
        # practice if the start and goal positions are sufficiently dispersed and
        # not collinear.
        #
        # Since we have our own collision avoidance, we use squared Euclidean
        # distance instead. Minimizing sum of squared distances is closer to
        # minimizing the # maximum distance for any robot, which generally is more
        # important in multi-robot applications. We could use e.g. Euclidean^4 to
        # approximate maximum distance even more closely.
        #
        if args.assign:
            dists = sp.spatial.distance.cdist(starts, goals, "sqeuclidean")
            _, assignments = sp.optimize.linear_sum_assignment(dists)
            goals = goals[assignments,:]

        for goal, cf in zip(goals, cfs):
            cf.goTo(goal, yaw=0.0, duration=duration)
        timeHelper.sleep(duration + 1.0)


if __name__ == "__main__":
    main()
