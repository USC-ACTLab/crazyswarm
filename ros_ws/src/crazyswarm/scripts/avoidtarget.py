#!/usr/bin/env python

from __future__ import print_function

from math import *
import numpy as np
import random

from pycrazyswarm import *

import rospy
from geometry_msgs.msg import PoseStamped
import time

SPREAD = 1.2

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cfs = allcfs.crazyflies

    pub = rospy.Publisher('virtual_interactive_object', PoseStamped, queue_size=10)
    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    msg.pose.position.x = -2.0
    msg.pose.position.y = 0.25
    msg.pose.position.z = 1.0
    #quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    msg.pose.orientation.x = 0 #quaternion[0]
    msg.pose.orientation.y = 0 #quaternion[1]
    msg.pose.orientation.z = 0 #quaternion[2]
    msg.pose.orientation.w = 1 #quaternion[3]

    print("press button to take off...")
    swarm.input.waitUntilButtonPressed()

    rand_heights = None

    if len(cfs) > 1:
        rand_heights = [random.random() for cf in cfs]
        lowest = min(rand_heights)
        highest = max(rand_heights)

        # ensure we fill up the full range of heights
        MIN_HEIGHT = 0.6
        MAX_HEIGHT = 1.6
        scale = (MAX_HEIGHT - MIN_HEIGHT) / (highest - lowest)

        for i in range(len(rand_heights)):
            rand_heights[i] = scale * (rand_heights[i] - lowest) + MIN_HEIGHT
    else:
        rand_heights = [1.0]

    heights = {}
    for cf, height in zip(cfs, rand_heights):
        heights[cf] = height
        duration = height + 1.0
        cf.takeoff(height, duration)

    timeHelper.sleep(max(rand_heights) + 1.0)

    for cf in cfs:
        hover_pos = cf.initialPosition * np.array([SPREAD, SPREAD, 1.0]) + np.array([0, 0, heights[cf]])
        cf.hover(hover_pos, 0, 2.0)

    timeHelper.sleep(2.5)


    print("press button to start avoiding...")
    swarm.input.waitUntilButtonPressed()

    MAX_DISPLACEMENT = 1.2 * 0.5 # TODO get spacing from init positions
    MAX_SPEED = 1.5 # m/s

    for cf in cfs:
        home = cf.initialPosition * np.array([SPREAD, SPREAD, 1.0]) + np.array([0, 0, heights[cf]])
        cf.avoidTarget(home, MAX_DISPLACEMENT, MAX_SPEED)

    # timeHelper.sleep(1.0)
    # allcfs.setParam("ring/headlightEnable", 1)


    print("press button to go home...")
    SPEED = 0.4 # m/s
    while not rospy.is_shutdown():
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x += SPEED * 0.01
        pub.publish(msg)

        if swarm.input.checkIfButtonIsPressed():
            break

        if msg.pose.position.x > 2.0 or msg.pose.position.x < -2.0:
            SPEED *= -1

        time.sleep(0.01)

    # swarm.input.waitUntilButtonPressed()
    # allcfs.setParam("ring/headlightEnable", 0)
    for cf in cfs:
        hover_pos = cf.initialPosition + np.array([0, 0, heights[cf]])
        cf.hover(hover_pos, 0, 2.0)

    timeHelper.sleep(2.5)


    print("press button to land...")
    swarm.input.waitUntilButtonPressed()
    allcfs.land(targetHeight = 0.02, duration = 3.5)


if __name__ == "__main__":
    main()

