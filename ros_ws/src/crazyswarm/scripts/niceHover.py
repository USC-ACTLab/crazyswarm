#!/usr/bin/env python

import time
import numpy as np

from crazyflie import *
from trajectory import *

if __name__ == "__main__":
    allcfs = CrazyflieServer()

    traj = Trajectory()
    traj.load("../launch/figure8_smooth.csv")
    traj.stretch(1.0)

    TRIALS = 1
    for i in range(TRIALS):
        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        time.sleep(2.0)
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
            cf.hover(pos, 0, 1.0)
        time.sleep(2.0)

        for cf in allcfs.crazyflies:
            cf.uploadTrajectory(traj)

	# stabilize after jittery traj upload
	time.sleep(4.0)

        allcfs.startTrajectory()
        time.sleep(traj.totalDuration() + 2.0)

        allcfs.land(targetHeight=0.06, duration=2.0)
        time.sleep(3.0)

