#!/usr/bin/env python

import time

from crazyflie import *
from trajectory import *

if __name__ == "__main__":
    allcfs = CrazyflieServer()

    pos1 = allcfs.crazyflies[0].position()
    pos2 = allcfs.crazyflies[1].position()
    print(pos1)
    print(pos2)

    allcfs.crazyflies[0].takeoff(targetHeight=1.0, duration=2.0)
    allcfs.crazyflies[1].takeoff(targetHeight=0.5, duration=2.0)
    time.sleep(2)

    allcfs.crazyflies[0].hover(goal=[pos2[0], pos2[1], 1.0], yaw=0, duration=2.0)
    allcfs.crazyflies[1].hover(goal=[pos1[0], pos1[1], 0.5], yaw=0, duration=2.0)
    time.sleep(2)

    allcfs.land(targetHeight=0.06, duration=2.0)
