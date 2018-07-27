#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *
import pycrazyswarm.cfsim.cffirmware as firm
import uav_trajectory

TIMESCALE = 0.8

POSITIONS = [
    [0,0.8,1.2],
    [0.25,0.4,1.9],
    [0.25,-0.4,1.9],
    [-0.25,0.4,0.5],
    [-0.25,-0.4,0.5],
    [0,-0.8,1.2]
]

OFFSET = [0.0, 0.0, 0.0]

TRAJMAPPING = {15: 1, 16: 2, 17: 3, 18: 4, 19: 5, 20: 6}

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    root = 'iros2018_swap6v_pps'

    T = 0
    for cf in allcfs.crazyflies:
        traj = uav_trajectory.Trajectory()
        fname = '{0}/cf{1}.csv'.format(root, TRAJMAPPING[cf.id])
        traj.loadcsv(fname)
        cf.uploadTrajectory(0, 0, traj)
        T = max(T, traj.duration)
    print("T: ", T * TIMESCALE)

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(2.5)

    for cf in allcfs.crazyflies:
        pos = POSITIONS[TRAJMAPPING[cf.id] - 1]
        print(pos)
        cf.goTo(np.array(pos) + np.array(OFFSET), 0, 3.0)
    timeHelper.sleep(3.5)

    allcfs.startTrajectory(0, timescale=TIMESCALE)
    timeHelper.sleep(T + 3.0)
    allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
    timeHelper.sleep(T + 3.0)

    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
        cf.goTo(pos, 0, 3.0)
    timeHelper.sleep(3.5)

    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)

