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
    traj.stretch(1.0)

    for id, cf in allcfs.crazyflies.iteritems():
        cf.uploadTrajectory(traj)

    allcfs.startTrajectory()
    time.sleep(traj.totalDuration())
    allcfs.land()
