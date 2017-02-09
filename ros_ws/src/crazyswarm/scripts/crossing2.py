#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *
import pycrazyswarm.cfsim.cffirmware as firm

SCALE = 0.25

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    ids = range(1, 5)

    cfs = [allcfs.crazyfliesById[i] for i in ids]
    root = 'crossing4_pps'
    fnames = ['{0}/pp{1}.csv'.format(root, i) for i in range(1, len(ids) + 1)]
    trajs = [piecewise.loadcsv(fname) for fname in fnames]

    for traj in trajs:
        firm.piecewise_stretchtime(traj, SCALE);

    for cf, traj in zip(cfs, trajs):
        cf.uploadTrajectory(traj)

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(2.5)

    for cf in cfs:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
        cf.hover(pos, 0, 2.0)
    timeHelper.sleep(2.5)

    allcfs.startTrajectory()
    timeHelper.sleep(15.0 * SCALE) # TODO...
    allcfs.startTrajectoryReversed()
    timeHelper.sleep(15.0 * SCALE) # TODO...

    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)

