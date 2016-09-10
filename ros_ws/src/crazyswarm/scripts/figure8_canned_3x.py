#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    TRIALS = 1
    for i in range(TRIALS):
        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.0)
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
            cf.hover(pos, 0, 2.0)
        timeHelper.sleep(2.0)

        for i in range(0, 3):
            allcfs.startCannedTrajectory(0, 0.8)
            timeHelper.sleep(8.0)

        allcfs.land(targetHeight=0.02, duration=3.0)
        timeHelper.sleep(3.0)
