#!/usr/bin/env python

from __future__ import print_function

import numpy as np
from pycrazyswarm import *


Z = 1.0
takeoffDuration = 3.0
sleepRate = 30
repeats = 1


def goCircle(timeHelper, cf, totalTime, radius, kPosition):
    startTime = timeHelper.time()
    pos = cf.position()
    startPos = cf.initialPosition + np.array([0, 0, Z])
    center_circle = startPos - np.array([radius, 0, 0])
    time = 0
    while time < totalTime:
        time = timeHelper.time() - startTime
        omega = 2 * np.pi / totalTime
        vx = -radius * omega * np.sin(omega * time)
        vy = radius * omega * np.cos(omega * time)
        desiredVel = [vx, vy, 0.0]
        desiredPos = center_circle + radius * np.array(
            [np.cos(omega * time), np.sin(omega * time), 0])
        errorX = desiredPos - cf.position()
        cf.cmdVelocityWorld(desiredVel + kPosition * errorX, yawRate=0)
        timeHelper.sleepForRate(sleepRate)


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cf = allcfs.crazyflies[0]
    print("using cf", cf.id)

    cf.takeoff(targetHeight=Z, duration=takeoffDuration)
    timeHelper.sleep(2 + Z)
    for _ in range(repeats):
        goCircle(timeHelper, cf, totalTime=6, radius=0.5, kPosition=1.0)
    cf.notifySetpointsStop(remainValidMillisecs=100)
    cf.land(targetHeight=0.04, duration=takeoffDur)
    timeHelper.sleep(takeoffDur + 1.0)
