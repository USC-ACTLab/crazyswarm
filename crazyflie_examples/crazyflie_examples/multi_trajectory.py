#!/usr/bin/env python

import numpy as np
from pathlib import Path

from crazyflie_py import *
from crazyflie_py.uav_trajectory import Trajectory

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    trajs = []
    n = 2 # number of distinct trajectories

    for i in range(n):
        traj = Trajectory()
        traj.loadcsv(Path(__file__).parent / f"data/multi_trajectory/traj{i}.csv")
        trajs.append(traj)

    TRIALS = 1
    TIMESCALE = 1.0
    for i in range(TRIALS):
        for idx, cf in enumerate(allcfs.crazyflies):
            cf.uploadTrajectory(0, 0, trajs[idx % len(trajs)])

        allcfs.takeoff(targetHeight=1.0, duration=0.1)
        timeHelper.sleep(0.2)
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, 0])
            cf.goTo(pos, 0, 1.0)
        timeHelper.sleep(1.1)

        allcfs.startTrajectory(0, timescale=TIMESCALE)
        timeHelper.sleep(max([t.duration for t in trajs]) * TIMESCALE + 0.2)

        allcfs.land(targetHeight=0.0, duration=0.5)
        timeHelper.sleep(0.6)


if __name__ == "__main__":
    main()
