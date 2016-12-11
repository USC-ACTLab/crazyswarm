#!/usr/bin/env python

from __future__ import print_function

from math import *
import numpy as np

from pycrazyswarm import *

PERIOD = 15

def main():

    pyramid_steps = [
        list(range( 1,  8)) + [ 8, 14, 15, 21, 22, 28, 29, 35, 36, 42] + list(range(43, 50)),
        list(range( 9, 14)) + [     16, 20, 23, 27, 30, 34      ] + list(range(37, 42)),
        [17, 18, 19, 24, 26, 31, 32, 33],
        [25]
    ]

    heights = [0.7, 1.2, 1.7, 2.2]
    groups = [1, 2, 3, 4]
    #heights = [0.7, 1.4, 2.1]

    # sanity checks
    # s = set()
    # for step in pyramid_steps:
    #   for i in step:
    #       s.add(i)

    # assert(len(s) == 49)
    # for i in range(1, 50):
    #   assert(i in s)

    # connect to the server
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cfs = allcfs.crazyflies

    # setup ellipse upfront & assign groups
    for step, height, group in zip(pyramid_steps, heights, groups):
        for i in step:
            cf = allcfs.crazyfliesById[i]
            major = cf.initialPosition
            minor = [-major[1], major[0], 0]
            cf.setEllipse(
                center = np.array([0, 0, height]),
                major  = major,
                minor  = minor,
                period = PERIOD)
            cf.setGroup(group)

    # takeoff sequence
    for step, height, group in reversed(zip(pyramid_steps, heights, groups)):
        print("press button to continue...")
        swarm.input.waitUntilButtonPressed()
        # takeoff
        allcfs.takeoff(height, 1.0 + height, group = group)
        timeHelper.sleep(1.2 + height)
        # go to ideal position
        for i in step:
            cf = allcfs.crazyfliesById[i]
            cf.hover(cf.initialPosition + np.array([0, 0, height]), 0, 1.0)

    print("press button to start rotation...")
    swarm.input.waitUntilButtonPressed()
    allcfs.startEllipse()

    print("press button to stop...")
    swarm.input.waitUntilButtonPressed()
    for step, height in zip(pyramid_steps, heights):
        for i in step:
            cf = allcfs.crazyfliesById[i]
            cf.hover(cf.initialPosition + np.array([0, 0, height]), 0, 2.0)
    timeHelper.sleep(2.5)

    for step, height, group in zip(pyramid_steps, heights, groups):
        print("press button to land...")
        swarm.input.waitUntilButtonPressed()
        allcfs.land(0.02, 1.5 + height, group = group)
        timeHelper.sleep(1.7 + height)

if __name__ == "__main__":
    main()

