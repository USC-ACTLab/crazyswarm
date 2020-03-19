"""Demonstration of Buffered Voronoi Cell collision avoidance algorithm."""

from __future__ import print_function
import argparse

import numpy as np
import quadprog
import scipy as sp
import scipy.spatial
import scipy.optimize

import pycrazyswarm
import pycrazyswarm.util as util
import pycrazyswarm.cfsim.cffirmware as firm


DURATION = 4.0  # When not using BVC, duration for goTo commands.
MAX_SPEED = 1.0  # Maximum desired speed the BVC controller will request.
Z = 1.0  # Takeoff altitude.
KP_POS = 4.0  # P-controller gain for position when using BVC.

# Select velocities such that if we follow the velocity for this many seconds,
# we will remain inside our Voronoi cell.
HORIZON = 1.0

# Run BVC controller this many times per second.
RATE = 10


class BVCController:
    def __init__(self, goal, radii, bbox_min, bbox_max):
        self.goal = goal
        self.radii = radii
        self.bbox_min = bbox_min
        self.bbox_max = bbox_max
        self.dt = 1.0 / RATE
        self.lastVelocity = np.zeros(3)

    def cmdVel(self, pos, other_positions, horizon):
        """Computes the velocity setpoint for the low-level controller."""
        to_goal = self.goal - pos
        goal_dist = np.linalg.norm(to_goal)
        goal_dir = to_goal / goal_dist

        to_neighbors = other_positions - pos
        neighbor_dists = np.linalg.norm(to_neighbors, axis=1)
        neighbor_dirs = to_neighbors / neighbor_dists[:,None]

        inners = np.dot(neighbor_dirs, goal_dir)
        in_way = (neighbor_dists <= goal_dist) & (inners > 0.0)
        if np.any(in_way):
            worst_inner = np.amax(inners[in_way])
            sidestep = util.normalize(np.cross(goal_dir, np.array([0, 0, 1])))
            # Allow big sidestep when goal and neighbor are almost collinear.
            sidestep_amount = worst_inner**4 * max(goal_dist, 2*self.radii[0])
            to_goal = to_goal + sidestep_amount * sidestep

        # The polytope constraints for my Voronoi cell.
        Acell, bcell = self._cell_constraints(to_neighbors)

        # Add constraints for the environment bounding box.
        Abox = np.vstack([np.eye(3), -np.eye(3)])
        bbox = np.concatenate([(self.bbox_max - pos), -(self.bbox_min - pos)])

        A = np.vstack([Acell, Abox])
        b = np.concatenate([bcell, bbox])

        # Solve the QP for closest velocity that will remain in cell.
        quad = np.eye(3)
        lin = to_goal
        result = quadprog.solve_qp(quad, lin, -horizon * A.T, -b)
        vel = result[0]

        vel = util.clamp_norm(vel, MAX_SPEED)
        self.lastVelocity = vel
        return vel

    def _cell_constraints(self, relative_positions):
        """Computes my buffered Voronoi cell in linear inequality form."""
        relative_positions = relative_positions / self.radii[None,:]
        n, dim = relative_positions.shape
        A = []
        b = []
        for i, pos in enumerate(relative_positions):
            distance = np.linalg.norm(pos)
            A.append(pos / (distance * self.radii))
            b.append(distance / 2 - 1.0)
        if len(b) > 0:
            return np.row_stack(A), np.array(b)
        else:
            return np.zeros((1, dim)), np.ones(1)



def formationChangeBVC(timeHelper, goals, cfs, radii, bbox_min, bbox_max):
    """Use BVC collision avoidance to execute formation change.

    Args:
        timeHelper: pycrazyswarm TimeHelper.
        goals (array float[n, 3]): goal positions.
        cfs (array Crazyflie[n]): Crazyflie objects.
        radii (array float[3]): Collision ellipsoid radii.

    Returns: None.
    """
    bvcs = [BVCController(goal, radii, bbox_min, bbox_max) for goal in goals]
    n = len(bvcs)
    while True:
        pos = np.stack([cf.position() for cf in cfs])
        goal_dists = np.linalg.norm(pos - goals, axis=1)
        if np.all(goal_dists < radii[0] / 5.0):
            return

        for i, (bvc, cf) in enumerate(zip(bvcs, cfs)):
            ind_others = list(range(i)) + list(range(i + 1, n))
            others = pos[ind_others]
            vel = bvc.cmdVel(pos[i], others, HORIZON)
            cf.cmdVelocityWorld(vel, yawRate=0.0)

        timeHelper.sleepForRate(RATE)


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--nobvc",
        help="Disable Buffered Voronoi Cell collision avoidance",
        action="store_true"
    )
    parser.add_argument(
        "--assign",
        help=("Use optimal (minimum sum of squared distances) cf-goal assignment"
            "instead of random assignment"),
        action="store_true"
    )
    parser.add_argument(
        "--loops",
        type=int,
        default=1,
        help="Repeat the experiment this many times, without resetting positions",
    )
    args, unknown = parser.parse_known_args()

    # Construct the Crazyswarm objects.
    rows, cols = 3, 5
    N = rows * cols
    crazyflies_yaml = util.grid_yaml(rows, cols, spacing=0.5)
    swarm = pycrazyswarm.Crazyswarm(crazyflies_yaml=crazyflies_yaml)
    timeHelper = swarm.timeHelper
    cfs = swarm.allcfs.crazyflies

    # Tell the visualizer to draw collision volume ellipsoids. Color indicates
    # if any collisions occur. Use --nobvc without --assign to see collisions.
    xy_radius = 0.125
    radii = xy_radius * np.array([1.0, 1.0, 3.0])
    timeHelper.visualizer.showEllipsoids(radii)

    swarm.allcfs.takeoff(targetHeight=Z, duration=Z+1.0)
    timeHelper.sleep(Z + 2.0)

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

        # --nobvc illustrates what will happen if we don't use collision avoidance.
        if args.nobvc:
            for goal, cf in zip(goals, cfs):
                cf.goTo(goal, yaw=0.0, duration=DURATION)
            timeHelper.sleep(DURATION)
        else:
            formationChangeBVC(timeHelper, goals, cfs, radii, bbox_min, bbox_max)


if __name__ == "__main__":
    main()
