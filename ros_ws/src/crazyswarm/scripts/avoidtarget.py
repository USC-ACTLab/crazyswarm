#!/usr/bin/env python

from __future__ import print_function

import time
from math import *
import numpy as np
import random

from crazyflie import *
from trajectory import *
import joystick

def main():
    server = CrazyflieServer()
    cfs = server.crazyflies
    joy = joystick.Joystick()


    print("press button to take off...")
    joy.waitUntilButtonPressed()

    heights = {}
    max_dur = 0
    for cf in cfs:
        height = random.uniform(0.4, 2.0)
        heights[cf] = height
        duration = height + 1.0
        max_dur = max(max_dur, duration)
        cf.takeoff(height, duration)

    time.sleep(max_dur)


    print("press button to start avoiding...")
    joy.waitUntilButtonPressed()

    MAX_DISPLACEMENT = 1.2 * 0.5 # TODO get spacing from init positions
    MAX_SPEED = 1.5 # m/s

    for cf in cfs:
        home = cf.initialPosition + np.array([0, 0, heights[cf]])
        cf.avoidTarget(home, MAX_DISPLACEMENT, MAX_SPEED)


    print("press button to go home...")
    joy.waitUntilButtonPressed()
    server.goHome()
    time.sleep(2.0) # TODO know how long it will take


    print("press button to land...")
    joy.waitUntilButtonPressed()
    server.land(targetHeight = 0.04, duration = 2.5)


if __name__ == "__main__":
    main()

