#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *
import pycrazyswarm.cfsim.cffirmware as firm
import uav_trajectory

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(2.5)

    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
        cf.goTo(np.array(pos), 0, 3.0)
    timeHelper.sleep(3.5)

    allcfs.land(targetHeight=0.17, duration=4.0)

