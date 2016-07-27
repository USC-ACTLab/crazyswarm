#!/usr/bin/env python

import sys
import yaml
from crazyflie_driver.srv import *

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

class Crazyflie:
    def __init__(self, id, pos):
        self.id = id
        self.pos = pos
        self.traj = None

    def uploadTrajectory(self, trajectory):
        self.traj = trajectory

    def setEllipse(self, center, major, minor, period):
        # TODO
        pass

    def takeoff(self, targetHeight, duration):
        # TODO
        pass

    def land(self, targetHeight, duration):
        # TODO
        pass

    def hover(self, goal, yaw, duration):
        # TODO
        pass

    def position(self):
        return self.pos

    def evaluate(self, t):
        return self.traj.evaluate(t)

class CrazyflieServer:
    def __init__(self):
        with open("../launch/crazyflies.yaml", 'r') as ymlfile:
            cfg = yaml.load(ymlfile)

        self.crazyflies = []
        for crazyflie in cfg["crazyflies"]:
            id = crazyflie["id"]
            pos = crazyflie["initialPosition"]
            self.crazyflies.append(Crazyflie(id, pos))

        self.fig = plt.figure()
        self.ax = fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-5,5])
        self.ax.set_ylim([-5,5])
        self.ax.set_zlim([0,3])
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.t = 0.0

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

    def sleep(self, duration):
        for t in np.arange(0, duration, 0.1):
            # x, y, z = traj.evaluate(self.t + t)
            # if oldPlot is not None:
            #     ax.collections.remove(oldPlot)
            # oldPlot = ax.scatter([x, x], [y, y + 1.0], [z + 1.0, z + 1.0])

            plt.pause(0.001)
            print(t)
        self.t += duration

