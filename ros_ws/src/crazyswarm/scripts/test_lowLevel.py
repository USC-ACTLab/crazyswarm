#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

Z = 1.0

def setUp():
    crazyflies_yaml = """
    crazyflies:
    - channel: 100
      id: 1
      initialPosition: [1.0, 0.0, 0.0]
    - channel: 100
      id: 10
      initialPosition: [0.0, -1.0, 0.0]
    """
    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml, args="--sim --vis null")
    timeHelper = swarm.timeHelper
    return swarm.allcfs, timeHelper

def test_cmdFullstate_zero_vel():
    allcfs, timeHelper = setUp()
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    cf = allcfs.crazyflies[0]

    pos = np.array(cf.initialPosition) + np.array([1, 1, Z])
    zeroVec = np.array([0.0,0.0,0.0])
    cf.cmdFullState(pos, zeroVec, zeroVec, 0, zeroVec)
    timeHelper.sleep(1.0)

    assert np.all(np.isclose(cf.position(), pos))

def test_cmdPosition_land():
    allcfs, timeHelper = setUp()
    cf = allcfs.crazyflies[0]
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    pos = np.array(cf.initialPosition) + np.array([1, 1, Z])
    cf.cmdPosition(pos,yaw=0.0)
    timeHelper.sleep(1.0)

    assert np.all(np.isclose(cf.position(), pos))

def test_cmdVelocityworld_check_velocity():
    allcfs, timeHelper = setUp()
    
    cf = allcfs.crazyflies[0]
    vel = np.array([1.0,1.0,1.0])
    cf.cmdVelocityWorld(vel, yawRate=0)
    timeHelper.sleep(1.0)

    assert np.all(np.isclose(cf.velocity(), vel))
    
def test_cmdVelocityworld_check_intergrate():
    allcfs, timeHelper = setUp()

    cf = allcfs.crazyflies[0]
    vel = np.ones(3)
    cf.cmdVelocityWorld(vel, yawRate=0)
    timeHelper.sleep(1.0)

    pos = cf.initialPosition + vel
    assert np.all(np.isclose(cf.position(), pos))

def test_cmdVelocityworld_disturbance():
    crazyflies_yaml = """
    crazyflies:
    - channel: 100
      id: 1
      initialPosition: [1.0, 0.0, 0.0]
    - channel: 100
      id: 10
      initialPosition: [0.0, -1.0, 0.0]
    """
    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml, args="--sim --vis null --disturbance 1.0")
    timeHelper = swarm.timeHelper

    cf = swarm.allcfs.crazyflies[0]

    vel = np.ones(3)
    cf.cmdVelocityWorld(vel, yawRate=0)
    timeHelper.sleep(1.0)

    pos = cf.initialPosition + vel
    assert not np.any(np.isclose(cf.position(), pos))