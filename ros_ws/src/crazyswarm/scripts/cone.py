#!/usr/bin/env python

from __future__ import print_function

import time
from math import *
import numpy as np

from crazyflie import *
from trajectory import *

def main():
    allcfs = CrazyflieServer()
    cfs = allcfs.crazyflies

    MAX_CFS = 1
    n_cfs = min(len(cfs), 1)

    n_cfs = 2

    VERT_STEP = 0.6
    RAD_STEP = 0.6
    MIN_RAD = 0.6
    PERIOD = 10

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
        ex.center = np.array([0, 0, z])
        cf.setEllipse(
            center = cf.center,
            major  = cf.radius * MAJ_AXIS,
            minor  = cf.radius * MIN_AXIS,
            period = PERIOD)

        print("cf {0}:\tcenter = {1}, major = {2}, minor = {3}".format(
            i, ex.ctr, ex.radius * MAJ_AXIS, ex.radius * MIN_AXIS))

        takeoff_dur = 2 * ex.z
        max_dur = max(max_dur, takeoff_dur)
        cf.takeoff(targetHeight = ex.z, duration = takeoff_dur)

    time.sleep(max_dur + 0.5)

    raw_input("press return to enter formation...")
    print("moving to formation")
    max_dur = 0
    for i in reversed(range(n_cfs)):
        cf = cfs[i]
        ex = extra[i]
        takeoff_pos = np.array(cf.initialPosition) + np.array([0, 0, ex.z])

        print("\ttakeoff pos:", takeoff_pos)

        ex.home = ex.center + ex.radius * MAJ_AXIS
        move_dist = np.linalg.norm(ex.home - takeoff_pos)
        move_dur = 2 * move_dist
        max_dur = max(max_dur, move_dur)

        print("\thovering to {0} in {1} sec".format(initial, move_dur))
        cf.hover(ex.home, 0, move_dur)

    time.sleep(max_dur + 0.5)

    raw_input("press return to start ellipse...")
    print("starting ellipse")
    allcfs.startEllipse()

    raw_input("press return to stop...")
    print("stopping")
    max_dur = 0
    for i in reversed(range(n_cfs)):
        cf = cfs[i]
        ex = ex[i]
        pos = np.array(cf.position())
        dist = np.linalg.norm(pos - ex.home)
        circumference = 2*pi * (MIN_RAD + i * RAD_STEP)
        speed = circumference / PERIOD
        # duration will be the same for all unless they are out of phase
        dur = 2 * dist / speed
        max_dur = max(dur, max_dur)
        print("cf {0}:\tcircumference = {1}, stop time = {2}".format(
            i, circumference, dur))
        cf.hover(ex.home, 0, dur)

    time.sleep(max_dur + 1)

    raw_input("press return to land...")
    print("landing")
    allcfs.land(targetHeight = 0.05, duration = 5)


if __name__ == "__main__":
    main()

