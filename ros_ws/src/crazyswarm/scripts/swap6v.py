#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *
import pycrazyswarm.cfsim.cffirmware as firm

SCALE = 1.0

POSITIONS = [
    [0, 1.0, 1.5],
    [0.25, 0.5, 2.37],
    [0.25, -0.5, 2.37],
    [-0.25, 0.5, 0.63],
    [-0.25, -0.5, 0.63],
    [0, -1.0, 1.5]
]


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    ids = range(1, 6+1)

    cfs = [allcfs.crazyfliesById[i] for i in ids]
    root = 'swap6v_pps'
    fnames = ['{0}/pp{1}.csv'.format(root, i) for i in range(1, len(ids) + 1)]
    trajs = [piecewise.loadcsv(fname) for fname in fnames]

    T = 0
    for traj in trajs:
        firm.piecewise_stretchtime(traj, SCALE);
        T = max(T, firm.piecewise_duration(traj))
    print("T: ", T)

    for cf, traj in zip(cfs, trajs):
        cf.uploadTrajectory(traj)

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(2.5)

    for cf, pos in zip(cfs, POSITIONS):
        cf.goTo(np.array(pos), 0, 3.0)
    timeHelper.sleep(3.5)

    allcfs.startTrajectory()
    timeHelper.sleep(T + 3.0)
    allcfs.startTrajectoryReversed()
    timeHelper.sleep(T + 3.0)

    for cf in cfs:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
        cf.goTo(pos, 0, 3.0)
    timeHelper.sleep(3.5)

    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)

