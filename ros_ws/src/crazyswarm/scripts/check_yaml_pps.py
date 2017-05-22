#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *
import pycrazyswarm.cfsim.cffirmware as firm

SCALE = 0.25

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    cf1 = allcfs.crazyfliesById[1]
    cf2 = allcfs.crazyfliesById[2]

    traj1 = piecewise.loadcsv('crossing2_pps/pp1.csv')
    traj2 = piecewise.loadcsv('crossing2_pps/pp2.csv')

    firm.piecewise_stretchtime(traj1, SCALE);
    firm.piecewise_stretchtime(traj2, SCALE);

    cf1.uploadTrajectory(traj1)
    cf2.uploadTrajectory(traj2)

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(2.5)
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
        cf.hover(pos, 0, 2.0)
    timeHelper.sleep(2.5)

    allcfs.startTrajectory()
    timeHelper.sleep(15.0 * SCALE) # TODO...
    allcfs.startTrajectoryReversed()
    timeHelper.sleep(15.0 * SCALE) # TODO...

    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)

