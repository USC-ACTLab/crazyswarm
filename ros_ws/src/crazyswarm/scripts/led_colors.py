#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *
import uav_trajectory

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # Configure the CFs so that the LED ring displays the solid color.
    # Overrides the launch file and the firmware default.
    for cf in allcfs.crazyflies:
        cf.setParam("ring/effect", 7)

    # Generate all possible on/off rgb values.
    rgb_bits = [tuple((x >> k) & 0x1 for k in range(3)) for x in range(8)]

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(2.5)

    TRIALS = 1
    for i in range(TRIALS):
        for rgb in rgb_bits:
            for cf in allcfs.crazyflies:
                cf.setLEDColor(*rgb)
            timeHelper.sleep(1.0)

