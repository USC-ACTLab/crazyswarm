#!/usr/bin/env python

import time

from crazyflie import *
from trajectory import *

if __name__ == "__main__":
    allcfs = CrazyflieServer()
    allcfs.takeoff()
    time.sleep(2)

    traj = Trajectory()
    traj.load("../launch/figure8_smooth.csv")
    allcfs.crazyflies["05"].uploadTrajectory(traj)

    allcfs.startTrajectory()
    time.sleep(10)

    allcfs.land()
