#!/usr/bin/env python

import numpy as np
import rospy

from pycrazyswarm import *
import pycrazyswarm.cfsim.cffirmware as firm
import uav_trajectory
from crazyflie_driver.msg import GenericLogData
from iros2018_swap6v import IROS2018_Swap6v
from iros2018_cf import IROS2018_CF

class Power:
  pass


class IROS2018_PowerCheck:

  VBAT_MAX = 3.9 # stop charging, start flying
  VBAT_MIN = 3.2 # stop flying, start charging

  def __init__(self, swarm):
    self.swarm = swarm
    self.timeHelper = swarm.timeHelper
    self.allcfs = swarm.allcfs

    for cf in self.allcfs.crazyflies:
      rospy.Subscriber("/cf{}/power".format(cf.id), GenericLogData, self.powerCallback, cf)
      cf.power = Power()
      cf.power.vbat = 0


  def powerCallback(self, data, cf):
    cf.power.vbat = data.values[0]
    cf.power.chargeCurrent = data.values[1]
    cf.power.state = data.values[2]
    cf.power.batteryLevel = data.values[3]
    # print(cf.id, cf.power.vbat, cf.power.chargeCurrent, cf.power.state, cf.power.batteryLevel)


  # all batteries above VBAT_MAX
  def readyToFly(self):
    allCharged = True
    for cf in self.allcfs.crazyflies:
      if cf.power.vbat < IROS2018_PowerCheck.VBAT_MAX:
        allCharged = False
        print("CF{} not ready to fly ({} V)".format(cf.id, cf.power.vbat))
    return True #allCharged


  # one battery below VBAT_MIN
  def needCharging(self):
    needCharging = False
    for cf in self.allcfs.crazyflies:
      if cf.power.vbat < IROS2018_PowerCheck.VBAT_MIN:
        needCharging = True
        break
    return needCharging


  def waitUntilBatteriesCharged(self):
    print("waitUntilBatteriesCharged...")
    while not rospy.is_shutdown():
      if self.readyToFly():
        print("Ready!")
        break
      self.timeHelper.sleep(1)


if __name__ == "__main__":
  swarm = Crazyswarm()
  for cf in swarm.allcfs.crazyflies:
    cf.trajectoryId = 0
    cf.pieceOffset = 0

  power = IROS2018_PowerCheck(swarm)
  demo_swap6v = IROS2018_Swap6v(swarm)
  demo_cf = IROS2018_CF(swarm)

  while not rospy.is_shutdown():
    print("Press Blue X to start swap6v or yellow Y to start cf...")

    runNext = None
    while not rospy.is_shutdown():
      if swarm.input.checkIfButtonIsPressed(2):
        runNext = 'swap6v'
        break
      elif swarm.input.checkIfButtonIsPressed(3):
        runNext = 'cf'
        break
    swarm.timeHelper.sleep(0.01)

    if power.readyToFly():
      if runNext == 'swap6v':
        demo_swap6v.runOnce()
      elif runNext == 'cf':
        demo_cf.runOnce()
    else:
      print("Couldn't take off: batteries too low!")

