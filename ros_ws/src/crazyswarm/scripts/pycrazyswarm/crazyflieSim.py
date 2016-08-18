#!/usr/bin/env python

import sys
import yaml
import math

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from trajectory import *

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


class Ellipse:
    def __init__(self, center, major, minor, period):
        self.center = np.array(center)
        self.major = np.array(major)
        self.minor = np.array(minor)
        self.period = period

    def evaluate(self, t):
        s = 2 * math.pi / self.period;
        cos_t = math.cos(s * t)
        sin_t = math.sin(s * t)
        return cos_t * self.major + sin_t * self.minor + self.center

class Crazyflie:
    def __init__(self, id, pos, timeHelper):
        self.id = id
        self.pos = pos
        self.initialPosition = np.array(pos)
        self.timeHelper = timeHelper
        self.traj = None
        self.startt = 0
        self.mode = 0 # 0 - traj, 1 - ellipse

    def uploadTrajectory(self, trajectory):
        self.traj = trajectory

    def setEllipse(self, center, major, minor, period):
        self.ellipse = Ellipse(center, major, minor, period)

    def takeoff(self, targetHeight, duration):
        self.traj = Trajectory()
        self.traj.load("takeoff.csv")
        pos = self.position()
        delta_z = targetHeight - pos[2]
        self.traj.stretch(duration / self.traj.totalDuration())
        self.traj.scale(1, 1, delta_z, 1)
        self.traj.shift(pos, 0)
        self.home = [pos[0], pos[1], targetHeight]
        self.startt = self.timeHelper.time()
        self.mode = 0

    def land(self, targetHeight, duration):
        self.traj = Trajectory()
        self.traj.load("takeoff.csv")
        pos = self.position()
        delta_z = targetHeight - pos[2]
        self.traj.stretch(duration / self.traj.totalDuration())
        self.traj.scale(1, 1, delta_z, 1)
        self.traj.shift(pos, 0)
        self.startt = self.timeHelper.time()
        self.mode = 0

    def hover(self, goal, yaw, duration):
        self.pos = np.array(goal)
        self.mode = 2

    def startTrajectory(self):
        self.startt = self.timeHelper.time()
        self.mode = 0

    def startEllipse(self):
        self.startt = self.timeHelper.time()
        self.mode = 1

    def startCannedTrajectory(self, trajectory, timescale):
        self.traj = Trajectory()
        if trajectory == 1:
            self.traj.load("figure8.csv")
        pos = self.position()
        self.traj.stretch(timescale)
        self.traj.shift(pos, 0)
        self.startt = self.timeHelper.time()
        self.mode = 0

    def goHome(self):
        # TODO
        pass

    def position(self):
        return self.pos

    def _update(self, t):
        if self.mode == 0 and self.traj is not None:
            self.pos = self.traj.evaluate(t - self.startt)
        elif self.mode == 1:
            self.pos = self.ellipse.evaluate(t - self.startt)

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
