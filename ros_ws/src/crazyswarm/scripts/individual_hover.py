#!/usr/bin/env python

from __future__ import print_function

import time

from crazyflie import *
import joystick

def main():
    server = CrazyflieServer()
    cfs = server.crazyflies
    joy = joystick.Joystick()

    for cf in cfs:
        print(cf.id)
        cf.takeoff(1.0, 2.5)
        print("press button to contine")
        joy.waitUntilButtonPressed()
        cf.land(0.04, 2.5)


if __name__ == "__main__":
    main()
