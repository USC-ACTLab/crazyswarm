#!/usr/bin/env python

import time

from crazyflie import *

if __name__ == "__main__":
    allcfs = CrazyflieServer()
    allcfs.takeoff()
    time.sleep(5)
    allcfs.land()
