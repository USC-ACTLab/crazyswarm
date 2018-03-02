#!/usr/bin/env python

import numpy as np
import math
import colorsys

from pycrazyswarm import *
import pycrazyswarm.cfsim.cffirmware as firm
from udp_multicast import UdpMulticastSender

SCALE = 8.0
#SHIFT = [-0.3, 0, 0] # shift by this amount after take-off
SHIFT = [0.0, 0, 0.75]

def firmVecToNp(vec):
    return np.array([vec.x, vec.y, vec.z])

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    sender = UdpMulticastSender()

    # Use this to custom-map trajectory IDs to CF IDs
    ids = [40]
    trajIds = [1]

    # ids = [cf.id for cf in allcfs.crazyflies]
    # trajIds = ids

    cfs = [allcfs.crazyfliesById[i] for i in ids]
    root = '/home/whoenig/heterogeneous/crazyswarm/ros_ws/src/crazyswarm/scripts/figure8_pps/'
    fnames = ['{0}/pp{1}.csv'.format(root, i) for i in trajIds] #range(1, len(ids) + 1)]
    trajs = [piecewise.loadcsv(fname) for fname in fnames]

    for traj in trajs:
        firm.piecewise_stretchtime(traj, SCALE)

    totalTime = 0
    for traj in trajs:
        totalTime = max(totalTime, firm.piecewise_duration(traj))

    # for cf, traj in zip(cfs, trajs):
    #     cf.uploadTrajectory(traj)

    hues = np.linspace(0,1.0,len(cfs))
    for cf, hue, traj in zip(cfs, hues, trajs):
        r,g,b = colorsys.hsv_to_rgb(hue, 0.9, 1.0)
        cf.setParam("ring/solidRed", int(r * 255))
        cf.setParam("ring/solidGreen", int(g * 255))
        cf.setParam("ring/solidBlue", int(b * 255))
        cf.uploadTrajectory(traj)

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(2.5)

    for cf, traj in zip(cfs, trajs):
        result = firm.piecewise_eval(traj, 0, 0.033)
        pos = firmVecToNp(result.pos) + np.array(SHIFT)
        # assert(math.fabs(pos[2] - 0.5) < 1e-3)
        cf.hover(pos, 0, 2.0)
    timeHelper.sleep(2.5)

    sender.send("startTrajectory")
    allcfs.startTrajectory()
    timeHelper.sleep(totalTime + 1.0)

    # allcfs.setParam("ring/headlightEnable", 1)
    # for cf, traj in zip(cfs, trajs):
    #     result = firm.piecewise_eval(traj, totalTime, 0.033)
    #     pos = firmVecToNp(result.pos)
    #     cf.hover(pos, math.pi, 2.0)
    
    # timeHelper.sleep(5.0)

    # for cf, traj in zip(cfs, trajs):
    #     result = firm.piecewise_eval(traj, totalTime, 0.033)
    #     pos = firmVecToNp(result.pos)
    #     cf.hover(pos, 0, 2.0)
    # timeHelper.sleep(2.5)
    # allcfs.setParam("ring/headlightEnable", 0)
    # timeHelper.sleep(0.5)

    timeHelper.sleep(5.0)

    allcfs.startTrajectoryReversed()
    timeHelper.sleep(totalTime + 1.0)

    for cf in cfs:
        hover_pos = cf.initialPosition + np.array([0, 0, 1.0])
        cf.hover(hover_pos, 0, 2.0)
    timeHelper.sleep(2.5)

    allcfs.land(targetHeight=0.02, duration=3.0)
    timeHelper.sleep(3.5)

