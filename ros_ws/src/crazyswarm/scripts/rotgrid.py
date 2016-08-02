#!/usr/bin/env python

from __future__ import print_function

import time
from math import *
import numpy as np

from crazyflie import *
from trajectory import *

import rospy
import joystick

Z = 1.75
PERIOD = 15

def setup(cfs):
    for cf in cfs:
        major = cf.initialPosition
        minor = [-major[1], major[0], 0]
        cf.setEllipse(
            center = np.array([0, 0, Z]),
            major  = major,
            minor  = minor,
            period = PERIOD)

def stop(cfs):
    for cf in cfs:
        pos = cf.initialPosition + np.array([0, 0, Z])
        cf.hover(pos, 0, 1)
    time.sleep(1.5)

def main():
    allcfs = CrazyflieServer()
    allcfs.takeoff(targetHeight=Z, duration=3)
    time.sleep(4)
    cfs = allcfs.crazyflies
    setup(cfs)
    allcfs.startEllipse()

    print("press button to go home")
    joy = joystick.Joystick()
    joy.waitUntilButtonPressed()
    stop(cfs)


if __name__ == "__main__":
    main()

