#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

Z = 1.5

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.crazyfliesById[9].setGroup(1)
    allcfs.crazyfliesById[10].setGroup(2)

    allcfs.takeoff(targetHeight=Z, duration=1.0 + Z, group = 1)
    timeHelper.sleep(1.5 + Z)
    allcfs.land(targetHeight=0.06, duration=1.0 + Z)
    timeHelper.sleep(1.5 + Z)
