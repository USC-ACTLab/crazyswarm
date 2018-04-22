#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    print("wait")
    swarm.input.waitUntilButtonPressed()

    allcfs.takeoff(targetHeight=2.0, duration=2.0)
    timeHelper.sleep(4.0)
    allcfs.crazyflies[0].hover([2.0, 2.0, 1.0], 0, 2)
    # allcfs.startCannedTrajectory(1, 1.0)
    timeHelper.sleep(10)
    allcfs.land(targetHeight = 0.04, duration=2.0)
    timeHelper.sleep(10)
