#!/usr/bin/env python

import numpy as np
import os

from pycrazyswarm import *
import pycrazyswarm.cfsim.cffirmware as firm
import uav_trajectory


class IROS2018_CF:

  TIMESCALE1 = 0.5
  TIMESCALE2 = 0.8
  OFFSET = [0.0, 0.0, 0.0]

  TRAJMAPPING = {1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6}

  def __init__(self, swarm):
    self.swarm = swarm
    self.timeHelper = swarm.timeHelper
    self.allcfs = swarm.allcfs

    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "iros2018_cf_pps")

    self.T1 = 0
    self.T2 = 0
    for cf in self.allcfs.crazyflies:
      traj1 = uav_trajectory.Trajectory()
      fname = '{0}/initial_to_c/cf{1}.csv'.format(root, self.TRAJMAPPING[cf.id])
      traj1.loadcsv(fname)
      self.traj1Id = cf.trajectoryId
      cf.uploadTrajectory(cf.trajectoryId, cf.pieceOffset, traj1)
      cf.trajectoryId += 1
      cf.pieceOffset += len(traj1.polynomials)
      self.T1 = max(self.T1, traj1.duration * self.TIMESCALE1)

      traj2 = uav_trajectory.Trajectory()
      fname = '{0}/c_to_f/cf{1}.csv'.format(root, self.TRAJMAPPING[cf.id])
      traj2.loadcsv(fname)
      self.traj2Id = cf.trajectoryId
      cf.uploadTrajectory(cf.trajectoryId, cf.pieceOffset, traj2)
      cf.trajectoryId += 1
      cf.pieceOffset += len(traj2.polynomials)
      self.T2 = max(self.T2, traj2.duration * self.TIMESCALE2)
    print("T1: ", self.T1)
    print("T2: ", self.T2)


  def runOnce(self, repeat = 1):
    self.allcfs.takeoff(targetHeight=0.5, duration=2.0)
    self.timeHelper.sleep(2.5)

    for cf in self.allcfs.crazyflies:
      pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
      cf.goTo(np.array(pos) + np.array(self.OFFSET), 0, 1.0)
    self.timeHelper.sleep(1.5)

    # initial -> C, wait 3 secs
    self.allcfs.startTrajectory(self.traj1Id, timescale=self.TIMESCALE1)
    self.timeHelper.sleep(self.T1 + 3.0)

    for i in range(0, repeat):
      # C->F, wait 3 secs
      self.allcfs.startTrajectory(self.traj2Id, timescale=self.TIMESCALE2)
      self.timeHelper.sleep(self.T2 + 3.0)

      # F->C, wait 3 secs
      self.allcfs.startTrajectory(self.traj2Id, timescale=self.TIMESCALE2, reverse=True)
      self.timeHelper.sleep(self.T2 + 3.0)

    # C -> initial
    self.allcfs.startTrajectory(self.traj1Id, timescale=self.TIMESCALE1, reverse=True)
    self.timeHelper.sleep(self.T1)

    # land
    for cf in self.allcfs.crazyflies:
      pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
      cf.goTo(pos, 0, 3.0)
    self.timeHelper.sleep(3.5)

    self.allcfs.land(targetHeight=0.06, duration=2.0)
    self.timeHelper.sleep(3.0)


if __name__ == "__main__":
  swarm = Crazyswarm()
  for cf in swarm.allcfs.crazyflies:
    cf.trajectoryId = 0
    cf.pieceOffset = 0

  demo_cf = IROS2018_CF(swarm)

  demo_cf.runOnce()
