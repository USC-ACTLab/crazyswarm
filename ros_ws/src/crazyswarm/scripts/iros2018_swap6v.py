#!/usr/bin/env python

import numpy as np
import rospy
import os

from pycrazyswarm import *
import pycrazyswarm.cfsim.cffirmware as firm
import uav_trajectory


class IROS2018_Swap6v:

  TIMESCALE = 0.5
  POSITIONS = [
      [0,0.7,1.2],
      [0.25,0.4,1.8],
      [0.25,-0.4,1.8],
      [-0.25,0.4,0.5],
      [-0.25,-0.4,0.5],
      [0,-0.7,1.2]
  ]
  OFFSET = [0.0, 0.0, 0.0]
  TRAJMAPPING = {1: 4, 2: 5, 3: 6, 4: 1, 5: 2, 6: 3}

  def __init__(self, swarm):
    self.swarm = swarm
    self.timeHelper = swarm.timeHelper
    self.allcfs = swarm.allcfs

    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "iros2018_swap6v_pps")

    self.T = 0
    for cf in self.allcfs.crazyflies:
      traj = uav_trajectory.Trajectory()
      fname = '{0}/cf{1}.csv'.format(root, self.TRAJMAPPING[cf.id])
      traj.loadcsv(fname)
      self.trajId = cf.trajectoryId
      cf.uploadTrajectory(cf.trajectoryId, cf.pieceOffset, traj)
      cf.trajectoryId += 1
      cf.pieceOffset += len(traj.polynomials)
      self.T = max(self.T, traj.duration * self.TIMESCALE)
    print("T: ", self.T)


  def runOnce(self, repeat = 1):
    self.allcfs.takeoff(targetHeight=1.0, duration=2.0)
    self.timeHelper.sleep(2.5)

    for cf in self.allcfs.crazyflies:
      pos = self.POSITIONS[self.TRAJMAPPING[cf.id] - 1]
      cf.goTo(np.array(pos) + np.array(self.OFFSET), 0, 3.0)
    self.timeHelper.sleep(3.5)

    for i in range(0, repeat):
      self.allcfs.startTrajectory(self.trajId, timescale=self.TIMESCALE)
      self.timeHelper.sleep(self.T + 1.0)
      self.allcfs.startTrajectory(self.trajId, timescale=self.TIMESCALE, reverse=True)
      self.timeHelper.sleep(self.T + 1.0)
      if rospy.is_shutdown():
        break

    for cf in self.allcfs.crazyflies:
      pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
      cf.goTo(pos, 0, 3.0)
    self.timeHelper.sleep(3.5)

    self.allcfs.land(targetHeight=0.06, duration=2.0)
    self.timeHelper.sleep(3.0)



if __name__ == "__main__":
  swarm = Crazyswarm()
  for cf in swarm.allcfs.crazyflies:
    cf.trajectoryId = 0
    cf.pieceOffset = 0

  # power = IROS2018_PowerCheck(swarm)
  swap6v = IROS2018_Swap6v(swarm)

  swap6v.runOnce()
