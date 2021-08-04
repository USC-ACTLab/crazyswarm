#!/usr/bin/env python

"""Tests for simulation-only functionality."""

import numpy as np

from pycrazyswarm import *


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


def test_attitudeRPY():
    """Checks differential flatness and roll/pitch/yaw calculations."""

    end = 0.99 * np.pi  # Not trying to deal with wrapping here.
    yaws = np.linspace(-end, end, 11)

    for yaw in yaws:
        allcfs, timeHelper = setUp()
        cf = allcfs.crazyflies[0]
        Z = 1.0

        cf.takeoff(targetHeight=Z, duration=1.0+Z)
        timeHelper.sleep(1.5+Z)
        cf.goTo(np.zeros(3), yaw=yaw, duration=1.0, relative=True)
        timeHelper.sleep(1.5)

        c = np.cos(yaw)
        s = np.sin(yaw)
        Ryaw = np.array([
            [c, -s, 0],
            [s,  c, 0],
            [0,  0, 1],
        ])
        forward, left, up = Ryaw.T

        dirAngleSigns = [
            ( forward, 1,  1),
            (-forward, 1, -1),
            (    left, 0, -1),
            (   -left, 0,  1),
        ]

        for direction, angleIdx, sign in dirAngleSigns:
            cf.goTo(direction, yaw=0.0, duration=1.0, relative=True)
            timeHelper.sleep(0.25)
            rpy = cf.rpy()
            assert rpy[angleIdx] * sign > np.radians(20)
            assert np.abs(rpy[1 ^ angleIdx]) < np.radians(0.001)
            assert np.abs(rpy[2] - yaw) < np.radians(0.001)
            timeHelper.sleep(1.0)
