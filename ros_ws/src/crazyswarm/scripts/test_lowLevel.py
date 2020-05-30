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
    """
    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml, args="--sim --vis null")
    timeHelper = swarm.timeHelper
    return swarm.allcfs, timeHelper

def test_cmdFullState_zeroVel():
    allcfs, timeHelper = setUp()
    cf = allcfs.crazyflies[0]

    pos = np.array(cf.initialPosition) + np.array([1, 1, Z])
    cf.cmdFullState(pos, np.zeros(3), np.zeros(3), 0, np.zeros(3))
    timeHelper.sleep(1.0)

    assert np.all(np.isclose(cf.position(), pos))

def test_cmdPosition():
    allcfs, timeHelper = setUp()
    cf = allcfs.crazyflies[0]

    pos = np.array(cf.initialPosition) + np.array([1, 1, Z])
    cf.cmdPosition(pos,yaw=0.0)
    timeHelper.sleep(1.0)

    assert np.all(np.isclose(cf.position(), pos))

def test_cmdVelocityWorld_checkVelocity():
    allcfs, timeHelper = setUp()
    
    cf = allcfs.crazyflies[0]
    vel = np.ones(3)
    cf.cmdVelocityWorld(vel, yawRate=0)
    timeHelper.sleep(1.0)

    assert np.all(np.isclose(cf.velocity(), vel))
    
def test_cmdVelocityWorld_checkIntegrate():
    allcfs, timeHelper = setUp()

    cf = allcfs.crazyflies[0]
    vel = np.ones(3)
    cf.cmdVelocityWorld(vel, yawRate=0)
    timeHelper.sleep(1.0)

    pos = cf.initialPosition + vel
    assert np.all(np.isclose(cf.position(), pos))

def test_cmdVelocityWorld_disturbance():
    crazyflies_yaml = """
    crazyflies:
    - channel: 100
      id: 1
      initialPosition: [1.0, 0.0, 0.0]
    """
    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml, args="--sim --vis null --disturbance 1.0")
    timeHelper = swarm.timeHelper

    cf = swarm.allcfs.crazyflies[0]

    vel = np.ones(3)
    cf.cmdVelocityWorld(vel, yawRate=0)
    timeHelper.sleep(1.0)

    pos = cf.initialPosition + vel
    assert not np.any(np.isclose(cf.position(), pos))