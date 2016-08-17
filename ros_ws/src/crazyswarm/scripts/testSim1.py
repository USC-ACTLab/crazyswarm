#!/usr/bin/env python

import numpy as np

from crazyflieSim import *

if __name__ == "__main__":
    timeHelper = TimeHelper()
    allcfs = CrazyflieServer(timeHelper)

    allcfs.takeoff(targetHeight=2.0, duration=2.0)
    timeHelper.sleep(4.0)
    allcfs.startCannedTrajectory(1, 1.0)
    timeHelper.sleep(10)
    allcfs.land(targetHeight = 0.04, duration=2.0)
    timeHelper.sleep(10)
