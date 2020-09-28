#!/usr/bin/env python

"""Execute pre-planned trajectories to swap sides in a hexagon formation.

This script works both as a unit test and with real hardware."""


import numpy as np

from pycrazyswarm import *
from pycrazyswarm import util
import pycrazyswarm.cfsim.cffirmware as firm
import uav_trajectory


TIMESCALE = 1.0

POSITIONS = [
    [0, 1.0, 1.5],
    [0.25, 0.5, 2.37],
    [0.25, -0.5, 2.37],
    [-0.25, 0.5, 0.63],
    [-0.25, -0.5, 0.63],
    [0, -1.0, 1.5]
]

OFFSET = [0.0, 0.0, -0.3]


def main(yaml=None, args=None):
    swarm = Crazyswarm(crazyflies_yaml=yaml, args=args)
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    trajIds = range(1, 7)
    cfs = allcfs.crazyflies[:6]
    root = 'swap6v_pps'
    fnames = ['{0}/pp{1}.csv'.format(root, i) for i in trajIds]

    T = 0
    for cf, fname in zip(cfs, fnames):
        traj = uav_trajectory.Trajectory()
        traj.loadcsv(fname)
        cf.uploadTrajectory(0, 0, traj)
        T = max(T, traj.duration)

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(2.5)

    for cf, trajId in zip(cfs, trajIds):
        pos = POSITIONS[trajId - 1]
        cf.goTo(np.array(pos) + np.array(OFFSET), 0, 3.0)
    timeHelper.sleep(3.5)

    allcfs.startTrajectory(0, timescale=TIMESCALE)
    timeHelper.sleep(T + 3.0)
    allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
    timeHelper.sleep(T + 3.0)

    for cf in cfs:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
        cf.goTo(pos, 0, 3.0)
    timeHelper.sleep(3.5)

    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)


def test_swap6v():
    initial_positions = np.array(POSITIONS)
    initial_positions[:, 2] = 0.0
    initial_positions = initial_positions.tolist()
    yaml = util.yaml_with_positions(initial_positions)
    print(yaml)
    main(yaml=yaml, args="--sim --vis null")


if __name__ == "__main__":
    main()
