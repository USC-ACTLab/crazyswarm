#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

if __name__ == "__main__":
    # execute waypoints
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    for i in range(0, 255):
        allcfs.crazyfliesById[1].setParam("glow/power", i)
        timeHelper.sleep(0.1)
