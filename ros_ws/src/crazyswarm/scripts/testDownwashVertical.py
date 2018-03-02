#!/usr/bin/env python

import numpy as np
import math

from pycrazyswarm import *
import pycrazyswarm.cfsim.cffirmware as firm

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    ids = [40, 2]
    cfs = [allcfs.crazyfliesById[i] for i in ids]

    print("press button to take off...")
    swarm.input.waitUntilButtonPressed()

    cfs[0].takeoff(targetHeight=2.0, duration=4.0)
    cfs[1].takeoff(targetHeight=0.5, duration=1.5)

    timeHelper.sleep(4.5)

    print("press button to move on top")
    swarm.input.waitUntilButtonPressed()

    ####################################

    # cfs[0].hover(np.array([0, 0, 2.0]), 0, 4.0)
    # cfs[1].hover(np.array([0, 1, 0.5]), 0, 4.0)

    # timeHelper.sleep(4.0)

    # pos = np.array([0, 0, 0.5])
    # while True:
    #     print("current pos", pos)
    #     print("press left/right shoulder to move up/down")
    #     buttons = swarm.input.waitUntilAnyButtonPressed()
    #     print(buttons)
    #     if buttons[5] == 1:
    #         pos += np.array([0, 0.0, 0.05])
    #     elif buttons[4] == 1:
    #         pos -= np.array([0, 0.0, 0.05])
    #     elif buttons[3] == 1:  # Yellow Y on x-box controller
    #         break
    #     if pos[1] == 1:
    #         pos[1] = -1
    #     else:
    #         pos[1] = 1
    #     cfs[1].hover(pos, 0, 4.0)
    #     timeHelper.sleep(4.0)

    ####################################

    pos = cfs[0].initialPosition + np.array([0.0, 0.0, 2.0])
    cfs[0].hover(pos, 0, 3.0)

    pos = cfs[0].initialPosition + np.array([0, 0, 0.5])
    cfs[1].hover(pos, 0, 3.0)
    timeHelper.sleep(3.0)

    
    while True:
        print("current pos", pos)
        print("press left/right shoulder to move up/down")
        buttons = swarm.input.waitUntilAnyButtonPressed()
        if buttons[5] == 1:
            pos += np.array([0, 0.0, 0.05])
        elif buttons[4] == 1:
            pos -= np.array([0, 0.0, 0.05])
        elif buttons[3] == 1:  # Yellow Y on x-box controller
            break
        cfs[1].hover(pos, 0, 1.0)
        timeHelper.sleep(1.0)

    ####################################


    cfs[1].hover(cfs[1].initialPosition + np.array([0, 0, 0.5]), 0, 4.0)
    timeHelper.sleep(4.5)
    cfs[1].land(targetHeight=0.02, duration=2.0)
    timeHelper.sleep(2.0)

    cfs[0].hover(cfs[0].initialPosition + np.array([0, 0, 0.5]), 0, 4.0)
    timeHelper.sleep(4.5)
    cfs[0].land(targetHeight=0.02, duration=2.0)
    timeHelper.sleep(2.0)
