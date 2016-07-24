#!/usr/bin/env python

import time

from crazyflie import *
from trajectory import *

if __name__ == "__main__":
    allcfs = CrazyflieServer()
    # allcfs.takeoff(targetHeight=1.0, duration=2.0)
    # time.sleep(2)
    # allcfs.land(targetHeight=0.06, duration=2.0)
    allcfs.crazyflies["07"].takeoff(targetHeight=1.5, duration=2.0)
    allcfs.crazyflies["02"].takeoff(targetHeight=1.0, duration=2.0)
    allcfs.crazyflies["04"].takeoff(targetHeight=1.5, duration=2.0)
    allcfs.crazyflies["05"].takeoff(targetHeight=1.0, duration=2.0)

    allcfs.crazyflies["06"].takeoff(targetHeight=1.0, duration=2.0)
    allcfs.crazyflies["08"].takeoff(targetHeight=1.5, duration=2.0)
    allcfs.crazyflies["09"].takeoff(targetHeight=1.0, duration=2.0)
    allcfs.crazyflies["01"].takeoff(targetHeight=1.5, duration=2.0)

    # time.sleep(2)
    # allcfs.land(targetHeight=0.06, duration=2.0)
