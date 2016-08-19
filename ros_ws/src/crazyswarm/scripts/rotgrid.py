#!/usr/bin/env python

from __future__ import print_function

from math import *
import numpy as np

from pycrazyswarm import *

Z = 1.0
PERIOD = 7

def setup(cfs):
    for cf in cfs:
        major = cf.initialPosition
        minor = [-major[1], major[0], 0]
        cf.setEllipse(
            center = np.array([0, 0, Z]),
            major  = major,
            minor  = minor,
            period = PERIOD)

def stop(cfs, timeHelper):
    for cf in cfs:
        pos = cf.initialPosition + np.array([0, 0, Z])
        cf.hover(pos, 0, 1)
    timeHelper.sleep(1.5)

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=3)
    timeHelper.sleep(4)
    cfs = allcfs.crazyflies
    setup(cfs)
    allcfs.startEllipse()

    print("press button to go home")
    swarm.input.waitUntilButtonPressed()

    stop(cfs, timeHelper)


if __name__ == "__main__":
    main()

