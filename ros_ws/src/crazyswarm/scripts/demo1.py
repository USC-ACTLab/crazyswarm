#!/usr/bin/env python

import time

from crazyflie import *
from trajectory import *

if __name__ == "__main__":
    allcfs = CrazyflieServer()
    allcfs.takeoff()
    time.sleep(2)

    # traj = Trajectory()
    # traj.load("../launch/figure8_smooth.csv")
    # traj.stretch(0.8)
    # allcfs.crazyflies["05"].uploadTrajectory(traj)

    cf = allcfs.crazyflies["05"]
    cf.setEllipse([0, 0, 1], [-1, 0, 0], [0, 0.5, 0], 10)

    allcfs.startEllipse()

    # allcfs.startTrajectory()
    # time.sleep(traj.totalDuration())

    # allcfs.land()
