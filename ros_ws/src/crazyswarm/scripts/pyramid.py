#!/usr/bin/env python

from __future__ import print_function

import time
from math import *
import numpy as np

from crazyflie import *
import rospy
import joystick

PERIOD = 20

def main():

	pyramid_steps = [
		list(range( 1,  8)) + [ 8, 14, 15, 21, 22, 28, 29, 35, 36, 42] + list(range(43, 50)),
		list(range( 9, 14)) + [		16, 20, 23, 27, 30, 34		] + list(range(37, 42)),
		[17, 18, 19, 24, 26, 31, 32, 33],
		[25]
	]

	heights = [0.7, 1.4, 2.1, 2.5]
	#heights = [0.7, 1.4, 2.1]

	# sanity checks
	# s = set()
	# for step in pyramid_steps:
	# 	for i in step:
	# 		s.add(i)

	# assert(len(s) == 49)
	# for i in range(1, 50):
	# 	assert(i in s)

	# connect to the server
	allcfs = CrazyflieServer()
	cfs = allcfs.crazyflies

	# setup ellipse upfront & assign groups
	group = 1
	for step, height in zip(pyramid_steps, heights):
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
		group += 1

	# takeoff sequence
	joy = joystick.Joystick()

	group = 1
	for step, height in reversed(zip(pyramid_steps, heights)):
		print("press button to continue...")
		joy.waitUntilButtonPressed()
		# takeoff
		allcfs.takeoff(height, 1.0 + height, group = group)
		time.sleep(1.2 + height)
		# go to ideal position
		for i in step:
			cf = allcfs.crazyfliesById[i]
			cf.hover(cf.initialPosition + np.array([0, 0, height]), 0, 1.0)
		group += 1

	print("press button to start rotation...")
	joy.waitUntilButtonPressed()
	allcfs.startEllipse()

	print("press button to stop...")
	joy.waitUntilButtonPressed()
	for step, height in zip(pyramid_steps, heights):
		for i in step:
			cf = allcfs.crazyfliesById[i]
			cf.hover(cf.initialPosition + np.array([0, 0, height]), 0, 2.0)
	time.sleep(2.5)

	group = 1
	for step, height in zip(pyramid_steps, heights):
		print("press button to land...")
		joy.waitUntilButtonPressed()
		allcfs.land(0.06, 1.0 + height, group = group)
		time.sleep(1.2 + height)
		group += 1

if __name__ == "__main__":
	main()

