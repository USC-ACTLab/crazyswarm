#!/usr/bin/env python

import numpy as np
import rospy
import math
from pycrazyswarm import *

RATE = 10
SPEED = 2.0 # m/s
CRAZYFLIE_ID = 49
PRINT_ONLY = False
r = 1.0
f = 0.4 * (2.0 * math.pi)

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # execute
    if not PRINT_ONLY:
        allcfs.takeoff(targetHeight=0.8, duration=2.0)
        timeHelper.sleep(2.0)

    lastUpdate = rospy.get_time()
    while not rospy.is_shutdown():
        t = rospy.get_time()
        goalPos = np.array([np.cos(t * f) * r, np.sin(t * f) * r, 0.8])
        if t - lastUpdate >= 1.0/RATE:
            cf = allcfs.crazyfliesById[CRAZYFLIE_ID]
            pos = cf.position()
            distance = np.linalg.norm(pos - goalPos)
            timeToGoal = max(distance / SPEED, 1.0)                
            print(goalPos,0.0,timeToGoal,distance)
            if not PRINT_ONLY:
                cf.hover(goalPos, 0.0, timeToGoal)
            lastUpdate = t
