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

def test_takeOff():
    allcfs, timeHelper = setUp()
    cf1 = allcfs.crazyflies[0]
    assert np.all(cf1.initialPosition == [1.0, 0.0, 0.0])
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    for cf in allcfs.crazyflies:
        pos = cf.initialPosition + np.array([0, 0, Z])
        assert np.all(np.isclose(cf.position(), pos,atol=0.001))

def test_goTo_nonRelative():
    allcfs, timeHelper = setUp()
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([1, 1, Z])
        cf.goTo(pos, 0, 1.0)
    timeHelper.sleep(1.0)

    #start testing
    for cf in allcfs.crazyflies:
        pos = cf.initialPosition + np.array([1, 1, Z])
        assert np.all(np.isclose(cf.position(), pos))

# test broadcasted relative goTo        
def test_goTo_relative():
    allcfs, timeHelper = setUp()
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    # broadcasted relative goTo
    allcfs.goTo(np.array([1.0,1.0,1.0]), 0, Z)
    timeHelper.sleep(2.0)

    #start testing
    for cf in allcfs.crazyflies:
        pos = cf.initialPosition + np.array([1.0,1.0,2*Z])
        assert np.all(np.isclose(cf.position(), pos))

def test_landing():
    allcfs, timeHelper = setUp()
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)

    #start testing
    for cf in allcfs.crazyflies:
        pos = cf.initialPosition + np.array([0, 0, 0.02])
        assert np.all(np.isclose(cf.position(), pos, atol=0.0001))