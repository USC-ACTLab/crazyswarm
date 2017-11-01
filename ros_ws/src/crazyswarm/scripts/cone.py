#!/usr/bin/env python

from __future__ import print_function

import time
from math import *
import numpy as np

from crazyflie import *
from trajectory import *

import rospy
from sensor_msgs.msg import Joy

class Object:
    pass

lastButtonState = 0
buttonWasPressed = False

def joyChanged(data):
    global buttonWasPressed
    global lastButtonState
    if not buttonWasPressed and data.buttons[5] == 1 and lastButtonState == 0:
        buttonWasPressed = True
    lastButtonState = data.buttons[5]

def waitUntilButtonPressed():
    global buttonWasPressed
    while not rospy.is_shutdown() and not buttonWasPressed:
        time.sleep(0.01)
    buttonWasPressed = False

def main():
    allcfs = CrazyflieServer()

    rospy.Subscriber("/joy", Joy, joyChanged)

    cfs = allcfs.crazyflies

    MAX_CFS = 1
    n_cfs = min(len(cfs), MAX_CFS)

    VERT_STEP = 1.0
    RAD_STEP = 0.3
    MIN_RAD = 2.0
    PERIOD = 15

    MAJ_AXIS = np.array([ 0, 1, 0])
    MIN_AXIS = np.array([-1, 0, 0])

    extra = [Object() for i in range(n_cfs)]

    # loop backwards so the highest CFs take off first
    print("taking off")
    max_dur = 0
    for i in reversed(range(n_cfs)):
        cf = cfs[i]
        ex = extra[i]
        ex.z = (i + 1) * VERT_STEP
        ex.radius = MIN_RAD + i * RAD_STEP
        ex.center = np.array([0, 0, ex.z])
        cf.setEllipse(
            center = ex.center,
            major  = ex.radius * MAJ_AXIS,
            minor  = ex.radius * MIN_AXIS,
            period = PERIOD)

        print("cf {0}:\tcenter = {1}, major = {2}, minor = {3}".format(
            i, ex.center, ex.radius * MAJ_AXIS, ex.radius * MIN_AXIS))

        takeoff_dur = 2 + 1 * ex.z
        max_dur = max(max_dur, takeoff_dur)
        cf.takeoff(targetHeight = ex.z, duration = takeoff_dur)

    time.sleep(max_dur + 0.5)

    print("press button to enter formation...")
    waitUntilButtonPressed()
    print("moving to formation")
    # highest CFs first
    for i in reversed(range(n_cfs)):
        cf = cfs[i]
        ex = extra[i]
        ex.takeoff_pos = np.array(cf.initialPosition) + np.array([0, 0, ex.z])

        print("\ttakeoff pos:", ex.takeoff_pos)

        ex.home = ex.center + ex.radius * MAJ_AXIS
        move_dist = np.linalg.norm(ex.home - ex.takeoff_pos)
        move_dur = 1 + 1 * move_dist
        max_dur = max(max_dur, move_dur)

        print("\thovering to {0} in {1} sec".format(ex.home, move_dur))
        cf.hover(ex.home, 0, move_dur)
        time.sleep(move_dur);

    print("press button to start ellipse...")
    waitUntilButtonPressed()
    print("starting ellipse")
    allcfs.startEllipse()

    print("press button to stop...")
    waitUntilButtonPressed()
    print("stopping")
    max_dur = 0
    for i in reversed(range(n_cfs)):
        cf = cfs[i]
        ex = extra[i]
        #pos = np.array(cf.position())
        #dist = np.linalg.norm(pos - ex.takeoff_pos)
        #circumference = 2*pi * (MIN_RAD + i * RAD_STEP)
        #speed = circumference / PERIOD
        #dur = 2 * dist / speed
        dur = 3.0
        max_dur = max(dur, max_dur)
        #print("cf {0}:\tcircumference = {1}, stop time = {2}".format(
        #    i, circumference, dur))
        cf.hover(ex.takeoff_pos, 0, dur)

    time.sleep(max_dur + 1)

    print("press button to land...")
    waitUntilButtonPressed()
    print("landing")
    allcfs.land(targetHeight = 0.05, duration = 5)


if __name__ == "__main__":
    main()

