#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *
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

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    ids = [15, 16, 17, 18, 19, 20] #[15, 16, 17] #range(1, 6+1)
    trajIds = [1, 2, 3, 4, 5, 6] #[1, 2, 3, 4, 5, 6]

    cfs = [allcfs.crazyfliesById[i] for i in ids]
    root = 'swap6v_pps'
    fnames = ['{0}/pp{1}.csv'.format(root, i) for i in trajIds]
    # trajs = [piecewise.loadcsv(fname) for fname in fnames]

    T = 0
    for cf, fname in zip(cfs, fnames):
        traj = uav_trajectory.Trajectory()
        traj.loadcsv(fname)
        cf.uploadTrajectory(0, 0, traj)
        T = max(T, traj.duration)
    print("T: ", T * TIMESCALE)

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(2.5)

    for cf, trajId in zip(cfs, trajIds):
        pos = POSITIONS[trajId - 1]
        print(pos)
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

