#!/usr/bin/env python

import numpy as np
import math

from pycrazyswarm import *
import pycrazyswarm.cfsim.cffirmware as firm

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    print("press button to take off...")
    swarm.input.waitUntilButtonPressed()

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(2.5)

    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
        cf.hover(pos, 0, 2.0)
    timeHelper.sleep(2.0)

    speed = 0.5 # m/s
    odd = True
    while True:
        print("press left/right shoulder to move slower/faster (or back on the joystick to land)")
        buttons = swarm.input.waitUntilAnyButtonPressed()
        print(buttons)
        if buttons[5] == 1:
            speed += 0.1
        else:
            speed -= 0.1
        print("current speed", speed)
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition)
            if odd:
                pos += np.array([-3.0, 0.0, 1.0])
            else:
                pos += np.array([0, 0.0, 1.0])
            cf.hover(pos, 0, 3.0 / speed)
        timeHelper.sleep(3.0 / speed)
        odd = not odd
