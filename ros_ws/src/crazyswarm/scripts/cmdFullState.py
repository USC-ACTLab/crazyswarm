#!/usr/bin/env python

import numpy as np
import rospy

from pycrazyswarm import *
import uav_trajectory

def executeTrajectory(file, reverse = False, rate = 100, offset=np.array([0,0,0])):
    traj = uav_trajectory.Trajectory()
    traj.loadcsv(file)
    rate = rospy.Rate(rate)

    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        t = (rospy.Time.now() - start_time).to_sec()
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

        rate.sleep()

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    rate = 100

    executeTrajectory("takeoff.csv", False, rate)
    executeTrajectory("figure8.csv", False, rate, np.array([0,0,0.5]))
    executeTrajectory("takeoff.csv", True, rate)

    for i in range(0, 100):
        for cf in allcfs.crazyflies:
            cf.cmdStop()
