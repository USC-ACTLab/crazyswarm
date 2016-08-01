#!/usr/bin/env python

from __future__ import print_function

import time
from math import *
import numpy as np

from crazyflie import *
from trajectory import *
import joystick

def main():
    server = CrazyflieServer()
    cfs = allcfs.crazyflies
    joy = joystick.Joystick()

    HEIGHT = 1.0
    MAX_DISPLACEMENT = 1.2 * 0.5 # TODO get spacing from init positions
    MAX_SPEED = 3.0 # m/s

    server.takeoff(targetHeight=1.0, duration=2.0)
    time.sleep(2.5)

    print("press button to start...")
    joy.waitUntilButtonPressed()

    for cf in cfs:
        home = cf.initialPosition + np.array([0, 0, HEIGHT])
        cf.avoidTarget(home, MAX_DISPLACEMENT, MAX_SPEED)

    print("press button to go home...")
    joy.waitUntilButtonPressed()
    server.goHome()
    time.sleep(2.0) # TODO know how long it will take

    print("press button to land...")
    joy.waitUnitlButtonPressed()
    server.land(targetHeight = 0.04, duration = 2)


if __name__ == "__main__":
    main()

