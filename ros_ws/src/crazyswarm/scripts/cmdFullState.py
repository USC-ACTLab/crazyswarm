#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *
import uav_trajectory

def executeTrajectory(timeHelper, file, reverse = False, rate = 100, offset=np.array([0,0,0])):
    traj = uav_trajectory.Trajectory()
    traj.loadcsv(file)

    start_time = timeHelper.time()
    while not timeHelper.isShutdown():
        t = timeHelper.time() - start_time
        print(t)
        if t > traj.duration:
            break

        if reverse:
            e = traj.eval(traj.duration - t)
        else:
            e = traj.eval(t)
        for cf in allcfs.crazyflies:
            cf.cmdFullState(
                e.pos + np.array(cf.initialPosition) + offset,
                e.vel,
                e.acc,
                e.yaw,
                e.omega)

        timeHelper.sleepForRate(rate)


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    rate = 100

    executeTrajectory(timeHelper, "takeoff.csv", False, rate)
    executeTrajectory(timeHelper, "figure8.csv", False, rate, np.array([0, 0, 0.5]))
    executeTrajectory(timeHelper, "takeoff.csv", True, rate)

    for i in range(0, 100):
        for cf in allcfs.crazyflies:
            cf.cmdStop()
