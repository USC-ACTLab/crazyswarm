#!/usr/bin/env python

from __future__ import print_function

from math import *
import numpy as np

from pycrazyswarm import *

class Object:
    pass

def normalize(v):
    return v / np.linalg.norm(v)

MAJ_AXIS = np.array([ 0, 1, 0])
MIN_AXIS = np.array([-1, 0, 0])
VERT_MIN = 0.5
VERT_STEP = 0.3
RAD_STEP = 0.3
RAD_MIN = 0.3
PERIOD = 10

timeHelper = None
swarm = None

# setup so they go from (low, small-radius) to (high, big-radius)
def setup(cfs, extra, axis_scale):
    for i, (cf, ex) in enumerate(zip(cfs, extra)):
        ex.z = VERT_MIN + i * VERT_STEP
        ex.radius = RAD_MIN + i * RAD_STEP
        ex.center = np.array([0, 0, ex.z])
        ex.takeoff_pos = cf.initialPosition + ex.center
        ex.home = ex.center + axis_scale * ex.radius * MAJ_AXIS
        cf.setEllipse(
            center = ex.center,
            major  = axis_scale * ex.radius * MAJ_AXIS,
            minor  = axis_scale * ex.radius * MIN_AXIS,
            period = PERIOD)

# take off all at once but with different durations
def takeoff(cfs, extra):
    print("taking off")
    max_dur = 0
    for cf, ex in zip(cfs, extra):
        takeoff_dur = 2 + 1 * ex.z
        max_dur = max(max_dur, takeoff_dur)
        cf.takeoff(targetHeight = ex.z, duration = takeoff_dur)
    timeHelper.sleep(max_dur + 0.5)

def formation_simultaneous(cfs, extra):
    print("moving to formation")
    max_dur = 0
    for cf, ex in zip(cfs, extra):
        move_dist = np.linalg.norm(ex.home - ex.takeoff_pos)
        move_dur = 1 + 1 * move_dist
        max_dur = max(max_dur, move_dur)
        cf.hover(ex.home, 0, move_dur)
    timeHelper.sleep(max_dur + 0.5)

def stop(cfs, extra):

    def wrong_side(cf, ex):
        p = normalize(cf.position() - ex.center)
        a = normalize(ex.home - ex.center)
        return np.dot(p, a) < 0.15

    while any(wrong_side(cf, ex) for cf, ex in zip(cfs, extra)):
        print("unaligned stop, please try again...")
        swarm.input.waitUntilButtonPressed()

    print("stopping")
    max_dur = 0
    for cf, ex in zip(cfs, extra):
        dist = np.linalg.norm(cf.position() - ex.takeoff_pos)
        # TODO: account for current velocity
        dur = 1 + 2 * dist
        max_dur = max(max_dur, dur)
        cf.hover(ex.takeoff_pos, 0, dur)
    timeHelper.sleep(max_dur + 0.5)

def main():
    global timeHelper
    global swarm
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    cfs = allcfs.crazyflies
    MAX_CFS = 10
    n_cfs = min(len(cfs), MAX_CFS)
    assert(n_cfs & 0x1 == 0)
    cfs = cfs[0:n_cfs]
    n_half = n_cfs / 2

    # sort by dot product with major axis
    init_formation_center = sum(cf.initialPosition for cf in cfs) / n_cfs
    dotmaj = lambda cf : -np.dot(cf.initialPosition - init_formation_center, MAJ_AXIS)
    cfs = sorted(cfs, key=dotmaj)
    extra = [Object() for cf in cfs]

    # reverse first half of sorted list to get "V" formation
    setup(reversed(cfs[:n_half]), reversed(extra[:n_half]), 1.0)
    setup(cfs[n_half:], extra[n_half:], -1.0)
    takeoff(cfs, extra)

    print("press button to enter formation...")
    swarm.input.waitUntilButtonPressed()
    formation_simultaneous(cfs, extra)

    print("press button to start ellipse...")
    swarm.input.waitUntilButtonPressed()
    print("starting ellipse")
    allcfs.startEllipse()

    print("press button to stop...")
    swarm.input.waitUntilButtonPressed()
    stop(cfs, extra)

    print("press button to land...")
    swarm.input.waitUntilButtonPressed()
    print("landing")
    allcfs.land(targetHeight = 0.04, duration = 4)
    timeHelper.sleep(4.0 + 0.5)


if __name__ == "__main__":
    main()

