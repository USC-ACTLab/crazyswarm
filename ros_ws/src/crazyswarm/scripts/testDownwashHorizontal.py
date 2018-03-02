#!/usr/bin/env python

import numpy as np
import math

from pycrazyswarm import *
import pycrazyswarm.cfsim.cffirmware as firm

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    ids = [2,40]
    cfs = [allcfs.crazyfliesById[i] for i in ids]

    print("press button to take off...")
    swarm.input.waitUntilButtonPressed()

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(2.5)

    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
        cf.hover(pos, 0, 2.0)
    timeHelper.sleep(2.0)

    pos = cfs[1].initialPosition + np.array([0, 0, 1.0])
    while True:
        print("current pos", pos)
        print("press left/right shoulder to move left/right")
        buttons = swarm.input.waitUntilAnyButtonPressed()
        print(buttons)
        if buttons[5] == 1:
            pos += np.array([0, 0.05, 0.0])
        else:
            pos -= np.array([0, 0.05, 0.0])
        cfs[1].hover(pos, 0, 1.0)
        timeHelper.sleep(1.0)
