#!/usr/bin/env python

import numpy as np
import rospy

from pycrazyswarm import *
import pycrazyswarm.cfsim.cffirmware as firm
import uav_trajectory
from crazyflie_driver.msg import GenericLogData

VBAT_MAX = 3.9 # stop charging, start flying
VBAT_MIN = 3.2 # stop flying, start charging

TIMESCALE = 0.5

POSITIONS = [
    [0,0.7,1.2],
    [0.25,0.4,1.8],
    [0.25,-0.4,1.8],
    [-0.25,0.4,0.5],
    [-0.25,-0.4,0.5],
    [0,-0.7,1.2]
]

OFFSET = [0.0, 0.0, 0.0]

TRAJMAPPING = {1: 4, 2: 5, 3: 6, 4: 1, 5: 2, 6: 3}

class Power:
    pass

def powerCallback(data, cf):
    cf.power.vbat = data.values[0]
    cf.power.chargeCurrent = data.values[1]
    cf.power.state = data.values[2]
    cf.power.batteryLevel = data.values[3]
    print(cf.id, cf.power.vbat, cf.power.chargeCurrent, cf.power.state, cf.power.batteryLevel)


def waitUntilBatteriesCharged(cfs):
    print("waitUntilBatteriesCharged...")
    while not rospy.is_shutdown():
        allCharged = True
        for cf in cfs:
            if cf.power.vbat < VBAT_MAX:
                allCharged = False
        if allCharged:
            print("Ready!")
            break
        timeHelper.sleep(1)


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    root = 'iros2018_swap6v_pps'

    T = 0
    for cf in allcfs.crazyflies:
        traj = uav_trajectory.Trajectory()
        fname = '{0}/cf{1}.csv'.format(root, TRAJMAPPING[cf.id])
        traj.loadcsv(fname)
        cf.uploadTrajectory(0, 0, traj)
        T = max(T, traj.duration * TIMESCALE)
        rospy.Subscriber("/cf{}/power".format(cf.id), GenericLogData, powerCallback, cf)
        cf.power = Power()
        cf.power.vbat = 0
    print("T: ", T)

    while not rospy.is_shutdown():
        waitUntilBatteriesCharged(allcfs.crazyflies)

        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5)

        for cf in allcfs.crazyflies:
            pos = POSITIONS[TRAJMAPPING[cf.id] - 1]
            cf.goTo(np.array(pos) + np.array(OFFSET), 0, 3.0)
        timeHelper.sleep(3.5)

        while not rospy.is_shutdown():
            allcfs.startTrajectory(0, timescale=TIMESCALE)
            timeHelper.sleep(T + 1.0)
            allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
            timeHelper.sleep(T + 1.0)

            # stop if any battery is below a threshold
            stop = False
            for cf in allcfs.crazyflies:
                if cf.power.vbat < VBAT_MIN:
                    stop = True
                    break
            if stop:
                break

        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
            cf.goTo(pos, 0, 3.0)
        timeHelper.sleep(3.5)

        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)

