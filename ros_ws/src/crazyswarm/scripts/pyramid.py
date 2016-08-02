#!/usr/bin/env python

from __future__ import print_function

import time
from math import *
import numpy as np

from crazyflie import *
import rospy
import joystick

def main():

	pyramid_steps = [
		list(range( 1,  8)) + [ 8, 14, 15, 21, 22, 28, 29, 35, 36, 42] + list(range(43, 50)),
		list(range( 9, 14)) + [		16, 20, 23, 27, 30, 34		] + list(range(37, 42)),
		[17, 18, 19, 24, 26, 31, 32, 33],
		[25]
	]

	heights = [0.7, 1.4, 2.1, 2.5]

	s = set()
	for step in pyramid_steps:
		for i in step:
			s.add(i)

	assert(len(s) == 49)
	for i in range(1, 50):
		assert(i in s)


	allcfs = CrazyflieServer()
	cfs = allcfs.crazyflies

	joy = joystick.Joystick()

	print(pyramid_steps)
	print(heights)

	print("checkpoint")
	for step, height in reversed(zip(pyramid_steps, heights)):
		print("press button to continue...")
		joy.waitUntilButtonPressed()
		for i in step:
			cf = cfs[i - 1]
			assert(int(cf.id) == i)
			cf.takeoff(height, 3.0)
		time.sleep(3.2)
		for i in step:
			cf = cfs[i - 1]
			cf.hover(cf.initialPosition + np.array([0, 0, height]), 0, 1.0)

	print("press button to land...")
	joy.waitUntilButtonPressed()
	allcfs.land(0.04, 3.5)

if __name__ == "__main__":
	main()

