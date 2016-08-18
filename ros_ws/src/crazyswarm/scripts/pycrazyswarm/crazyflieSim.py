#!/usr/bin/env python

import sys
import os
import yaml
import math

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from trajectory import *

import cfsim.cffirmware as cfswig

def arr2vec(a):
    return cfswig.mkvec(a[0], a[1], a[2])

class TimeHelper:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-5,5])
        self.ax.set_ylim([-5,5])
        self.ax.set_zlim([0,3])
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.t = 0.0
        self.crazyflies = []
        self.oldPlot = None

    def time(self):
        return self.t

    def sleep(self, duration):
        for t in np.arange(0, duration, 0.1):
            xs = []
            ys = []
            zs = []
            for cf in self.crazyflies:
                cf._update(self.t + t)
                x, y, z = cf.position()
                xs.append(x)
                ys.append(y)
                zs.append(z)

            if self.oldPlot is not None:
                self.ax.collections.remove(self.oldPlot)
            self.oldPlot = self.ax.scatter(xs, ys, zs)
            plt.pause(0.001)
            print(t)
        self.t += duration

    def addObserver(self, observer):
        self.observers.append(observer)


# class Ellipse:
#     def __init__(self, center, major, minor, period):
#         self.center = np.array(center)
#         self.major = np.array(major)
#         self.minor = np.array(minor)
#         self.period = period

#     def evaluate(self, t):
#         s = 2 * math.pi / self.period;
#         cos_t = math.cos(s * t)
#         sin_t = math.sin(s * t)
#         return cos_t * self.major + sin_t * self.minor + self.center

class Crazyflie:
    def __init__(self, id, pos, timeHelper):
        self.id = id
        self.pos = pos
        self.initialPosition = np.array(pos)
        self.timeHelper = timeHelper

        MASS = 0.032
        self.planner = cfswig.planner()
        cfswig.plan_init(self.planner, MASS)
        self.planner.home = arr2vec(self.initialPosition) # TODO...

    # def uploadTrajectory(self, trajectory):
    #     self.traj = trajectory

    def setEllipse(self, center, major, minor, period):
        e = self.planner.ellipse
        e.center = arr2vec(center)
        e.major = arr2vec(major)
        e.minor = arr2vec(minor)
        e.period = period

    def takeoff(self, targetHeight, duration):
        t = self.timeHelper.time()
        pos = self._vposition(t)
        cfswig.plan_takeoff(self.planner,
            self._vposition(t), self.yaw(t), targetHeight, duration, t)

    def land(self, targetHeight, duration):
        t = self.timeHelper.time()
        cfswig.plan_land(self.planner,
            self._vposition(t), self.yaw(t), targetHeight, duration, t)

    def hover(self, goal, yaw, duration):
        t = self.timeHelper.time()
        cfswig.plan_hover(self.planner, arr2vec(goal), yaw, duration, t)

    def avoidTarget(self, home, maxDisplacement, maxSpeed):
        t = self.timeHelper.time()
        cfswig.plan_start_avoid_target(self.planner,
            arr2vec(home), maxDisplacement, maxSpeed, t)

    def startTrajectory(self):
        t = self.timeHelper.time()
        cfswig.plan_start_poly(self.planner, self._vposition(t), t)

    def startEllipse(self):
        t = self.timeHelper.time()
        cfswig.plan_start_ellipse(self.planner, t)

    # def startCannedTrajectory(self, trajectory, timescale):
    #     self.traj = Trajectory()
    #     if trajectory == 1:
    #         self.traj.load(os.path.join(os.path.dirname(__file__), "figure8.csv"))
    #     pos = self.position()
    #     self.traj.stretch(timescale)
    #     self.traj.shift(pos, 0)
    #     self.startt = self.timeHelper.time()
    #     self.mode = 0

    def goHome(self):
        t = self.timeHelper.time()
        for cf in self.crazyflies:
            cf.goHome(t)

    def position(self):
        return self.pos

    def yaw(self, t):
        ev = cfswig.plan_current_goal(self.planner, t)
        return ev.yaw

# "private" methods

    def _vposition(self, t):
        # TODO this should be implemented in C
        if self.planner.state == cfswig.TRAJECTORY_STATE_IDLE:
            return self.planner.home
        else:
            ev = cfswig.plan_current_goal(self.planner, t)
            # not totally sure why, but if we don't do this, we don't actually return by value
            return cfswig.mkvec(ev.pos.x, ev.pos.y, ev.pos.z)

    def _update(self, t):
        pos = self._vposition(t)
        self.pos = np.array([pos.x, pos.y, pos.z])

class CrazyflieServer:
    def __init__(self, timeHelper):
        with open("../launch/crazyflies.yaml", 'r') as ymlfile:
            cfg = yaml.load(ymlfile)

        self.crazyflies = []
        for crazyflie in cfg["crazyflies"]:
            id = crazyflie["id"]
            pos = crazyflie["initialPosition"]
            self.crazyflies.append(Crazyflie(id, pos, timeHelper))

        self.timeHelper = timeHelper
        self.timeHelper.crazyflies = self.crazyflies

    def emergency(self):
        # TODO
        pass

    def takeoff(self, targetHeight, duration):
        for crazyflie in self.crazyflies:
            crazyflie.takeoff(targetHeight, duration)

    def land(self, targetHeight, duration):
        for crazyflie in self.crazyflies:
            crazyflie.land(targetHeight, duration)

    def startTrajectory(self):
        for crazyflie in self.crazyflies:
            crazyflie.startTrajectory()

    def startEllipse(self):
        for crazyflie in self.crazyflies:
            crazyflie.startEllipse()

    def startCannedTrajectory(self, trajectory, timescale):
        for crazyflie in self.crazyflies:
            crazyflie.startCannedTrajectory(trajectory, timescale)

    def goHome(self):
        for crazyflie in self.crazyflies:
            crazyflie.goHome()
