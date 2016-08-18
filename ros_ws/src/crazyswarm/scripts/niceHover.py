#!/usr/bin/env python

import time
import numpy as np

from crazyflie import *

Z = 1.5

if __name__ == "__main__":
	allcfs = CrazyflieServer()

	allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
	time.sleep(1.5+Z)
	for cf in allcfs.crazyflies:
		pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
		cf.hover(pos, 0, 1.0)
