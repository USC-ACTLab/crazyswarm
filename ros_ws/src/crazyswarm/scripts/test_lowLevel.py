#!/usr/bin/env python

import numpy as np
import pytest

from pycrazyswarm import *


Z = 1.0


@pytest.fixture
def setUp(crazyswarm_ctor):
    crazyflies_yaml = """
    crazyflies:
    - channel: 100
      id: 1
      initialPosition: [1.0, 0.0, 0.0]
    """
    def setup(args=""):
        swarm = crazyswarm_ctor(crazyflies_yaml=crazyflies_yaml, args=args)
        timeHelper = swarm.timeHelper
        return swarm.allcfs, timeHelper
    return setup


def test_cmdFullState_zeroVel(setUp):
    allcfs, timeHelper = setUp()
    cf = allcfs.crazyflies[0]

    pos = np.array(cf.initialPosition) + np.array([1, 1, Z])
    cf.cmdFullState(pos, np.zeros(3), np.zeros(3), 0, np.zeros(3))
    timeHelper.sleep(1.0)

    assert np.all(np.isclose(cf.position(), pos))

def test_cmdPosition(setUp):
    allcfs, timeHelper = setUp()
    cf = allcfs.crazyflies[0]

    pos = np.array(cf.initialPosition) + np.array([1, 1, Z])
    cf.cmdPosition(pos,yaw=0.0)
    timeHelper.sleep(1.0)

    assert np.all(np.isclose(cf.position(), pos))

def test_cmdVelocityWorld_checkVelocity(setUp):
    allcfs, timeHelper = setUp()
    
    cf = allcfs.crazyflies[0]
    vel = np.ones(3)
    cf.cmdVelocityWorld(vel, yawRate=0)
    timeHelper.sleep(1.0)

    assert np.all(np.isclose(cf.velocity(), vel))
    
def test_cmdVelocityWorld_checkIntegrate(setUp):
    allcfs, timeHelper = setUp()

    cf = allcfs.crazyflies[0]
    vel = np.ones(3)
    cf.cmdVelocityWorld(vel, yawRate=0)
    timeHelper.sleep(1.0)

    pos = cf.initialPosition + vel
    assert np.all(np.isclose(cf.position(), pos))

def test_cmdVelocityWorld_disturbance(setUp):
    allcfs, timeHelper = setUp(args="--disturbance 1.0")
    cf = allcfs.crazyflies[0]

    vel = np.ones(3)
    cf.cmdVelocityWorld(vel, yawRate=0)
    timeHelper.sleep(1.0)

    pos = cf.initialPosition + vel
    assert not np.any(np.isclose(cf.position(), pos))

def test_sleepResidual(setUp):
    """Verify TimeHelper's time() is consistent with its integration steps."""
    np.random.seed(0)
    TRIALS = 100
    for _ in range(TRIALS):
        dtTick = 10 ** np.random.uniform(-2, 0)
        dtSleep = 10 ** np.random.uniform(-2, 0)
        allcfs, timeHelper = setUp("--dt {}".format(dtTick))

        cf = allcfs.crazyflies[0]
        vel = np.ones(3)
        cf.cmdVelocityWorld(vel, yawRate=0)
        time = 0.0
        while timeHelper.time() < 1.0:
            timeHelper.sleep(dtSleep)
            time += dtSleep

        assert time >= timeHelper.time()

        # We don't expect them to be exactly the same because timeHelper.time()
        # will always be an integer multiple of dtTick. However, we should not
        # be off by more than a tick.
        assert time - timeHelper.time() < dtTick

        pos = cf.initialPosition + timeHelper.time() * vel
        assert np.all(np.isclose(cf.position(), pos))

