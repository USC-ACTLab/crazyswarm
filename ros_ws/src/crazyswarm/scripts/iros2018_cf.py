#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *
import pycrazyswarm.cfsim.cffirmware as firm
import uav_trajectory

TIMESCALE = 0.8
OFFSET = [0.0, 0.0, 0.0]

TRAJMAPPING = {15: 1, 16: 2, 17: 3, 18: 4, 19: 5, 20: 6}

REPEAT = 1

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    root = 'iros2018_cf_pps'

    T1 = 0
    T2 = 0
    for cf in allcfs.crazyflies:
        traj1 = uav_trajectory.Trajectory()
        fname = '{0}/initial_to_c/cf{1}.csv'.format(root, TRAJMAPPING[cf.id])
        traj1.loadcsv(fname)
        cf.uploadTrajectory(0, 0, traj1)
        T1 = max(T1, traj1.duration)
        
        traj2 = uav_trajectory.Trajectory()
        fname = '{0}/c_to_f/cf{1}.csv'.format(root, TRAJMAPPING[cf.id])
        traj2.loadcsv(fname)
        cf.uploadTrajectory(1, len(traj1.polynomials), traj2)
        T2 = max(T2, traj2.duration)
    print("T1: ", T1 * TIMESCALE)
    print("T2: ", T2 * TIMESCALE)

    allcfs.takeoff(targetHeight=0.5, duration=2.0)
    timeHelper.sleep(2.5)

    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
        print(pos)
        cf.goTo(np.array(pos) + np.array(OFFSET), 0, 3.0)
    timeHelper.sleep(3.5)

    # initial -> C, wait 3 secs
    allcfs.startTrajectory(0, timescale=TIMESCALE)
    timeHelper.sleep(T1 + 3.0)

    for i in range(0, REPEAT):
        # C->F, wait 3 secs
        allcfs.startTrajectory(1, timescale=TIMESCALE)
        timeHelper.sleep(T2 + 3.0)

        # F->C, wait 3 secs
        allcfs.startTrajectory(1, timescale=TIMESCALE, reverse=True)
        timeHelper.sleep(T2 + 3.0)

    # C -> initial
    allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
    timeHelper.sleep(T1)

    # land
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
        cf.goTo(pos, 0, 3.0)
    timeHelper.sleep(3.5)

    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)

