#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

if __name__ == "__main__":
    # execute waypoints
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # test a single CF
    for i in range(0, 255):
        allcfs.crazyfliesById[1].setParam("glow/power", i)
        timeHelper.sleep(0.1)

    # test broadcasting (all)
    allcfs.crazyfliesById[1].setGroup(1)
    allcfs.crazyfliesById[2].setGroup(1)
    allcfs.crazyfliesById[3].setGroup(2)
    allcfs.crazyfliesById[4].setGroup(2)

    for i in range(0, 255):
        allcfs.setParam("glow/power", i)
        timeHelper.sleep(0.1)


    # test broadcasting (group 1)
    for i in range(0, 255):
        allcfs.setParam("glow/power", i, group = 1)
        allcfs.setParam("glow/power", 255 - i, group = 2)
        timeHelper.sleep(0.1)
