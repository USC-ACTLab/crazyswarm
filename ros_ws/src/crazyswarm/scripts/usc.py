#!/usr/bin/env python

# import time
import os
import numpy as np

from pycrazyswarm import *
# from crazyflie import *
# from sensor_msgs.msg import Joy

DY = 0.4 #m
DX = 0.3 #m
DZ = 0.3 #m
OX = 1
OY = 0.66
OZ = 0.5
DURATION = 1.4
SLEEP = 0.5
TRIALS = 100

# wantToExit = False

def hover(cf, pos, duration):
    cf.hover(
        np.array(pos) * np.array([DX, DY, DZ])
        + np.array([OX, OY, OZ])
        , 0, duration)

# def joyChanged(data):
#     global wantToExit
#     if data.buttons[5] == 1:
#         wantToExit = True


def sleep(duration):
    start = timeHelper.time()
    while True:
        now = timeHelper.time()
        elapsed = now - start
        if elapsed > duration:
            break
        if swarm.input.checkIfButtonIsPressed():
            for cf in allcfs.crazyflies:
                cf.hover(np.array(cf.initialPosition) + np.array([0, 0, 1]), 0, 3.0)
            timeHelper.sleep(4.0)
            allcfs.land(targetHeight = 0.04, duration = 2.0)
            timeHelper.sleep(2.0)
            os._exit(0)
        timeHelper.sleep(0.01)

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # print("wait")
    # swarm.input.waitUntilButtonPressed()


    # allcfs = CrazyflieServer()
    # rospy.Subscriber("/joy", Joy, joyChanged)

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    sleep(2.0)

    cfs = allcfs.crazyflies

    for i in range(0, TRIALS):

        hover(cfs[0], [0, 0, 5], DURATION)
        hover(cfs[1], [0, -3, 5], DURATION)
        hover(cfs[2], [-1, 0, 4], DURATION)
        hover(cfs[3], [-1, -3, 4], DURATION)
        hover(cfs[4], [-2, 0, 3], DURATION)
        hover(cfs[5], [-2, -3, 3], DURATION)
        hover(cfs[6], [-3, 0, 2], DURATION)
        hover(cfs[7], [-3, -1, 1], DURATION)
        hover(cfs[8], [-3, -2, 1], DURATION)
        hover(cfs[9], [-3, -3, 2], DURATION)


        sleep(DURATION + SLEEP)

        hover(cfs[0], [0, -1, 5], DURATION)
        # hover(cfs[1], [0, -3, 5], DURATION)
        # hover(cfs[2], [-1, 0, 4], DURATION)
        hover(cfs[3], [0, -2, 5], DURATION)
        hover(cfs[4], [-2, -1, 3], DURATION)
        hover(cfs[5], [-2, -2, 3], DURATION)
        hover(cfs[6], [-3, 0, 1], DURATION)
        # hover(cfs[7], [-3, -1, 1], DURATION)
        # hover(cfs[8], [-3, -2, 1], DURATION)
        # hover(cfs[9], [-3, -3, 2], DURATION)

        sleep(DURATION + SLEEP)

        hover(cfs[0], [0, -0.75, 5], DURATION)
        hover(cfs[1], [0, -2.25, 5], DURATION)
        # hover(cfs[2], [-1, 0, 4], DURATION)
        hover(cfs[3], [0, -1.5, 5], DURATION)
        hover(cfs[4], [-2, 0, 3], DURATION)
        hover(cfs[5], [-1, -3, 5], DURATION)
        hover(cfs[6], [-3, 0, 2], DURATION)
        # hover(cfs[7], [-3, -1, 1], DURATION)
        # hover(cfs[8], [-3, -2, 1], DURATION)
        hover(cfs[9], [-3, -3, 1], DURATION)

        sleep(DURATION + SLEEP)



# 0    1
# 2    3
# 4    5
# 6    9
#  7 8

#   0 3 1
# 2
#   4 5
#       9
# 6 7 8

#   0 3 1
# 2       5
# 4
# 6
#   7 8 9
