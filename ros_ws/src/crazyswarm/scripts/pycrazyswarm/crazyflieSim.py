#!/usr/bin/env python

import yaml
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import cfsim.cffirmware as firm

# main class of simulation.
# crazyflies keep reference to this object to ask what time it is.
# also does the plotting.
#
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

    def step(self, duration):
        self.t += duration

    # should be called "animate" or something
    # but called "sleep" for source-compatibility with real-robot scripts
    def sleep(self, duration):
        dt = 0.1
        for t in np.arange(self.t, self.t + duration, dt):
            xs = []
            ys = []
            zs = []
            for cf in self.crazyflies:
                x, y, z = cf.position()
                xs.append(x)
                ys.append(y)
                zs.append(z)

            if self.oldPlot is not None:
                self.ax.collections.remove(self.oldPlot)
            self.oldPlot = self.ax.scatter(xs, ys, zs)
            plt.pause(0.001)
            print(self.t)
            self.step(dt)


    def addObserver(self, observer):
        self.observers.append(observer)


# helper func to convert python/numpy arrays to firmware 3d vector type.
def arr2vec(a):
    return firm.mkvec(a[0], a[1], a[2])


class Crazyflie:

    def __init__(self, id, initialPosition, timeHelper):
        self.id = id
        self.initialPosition = np.array(initialPosition)
        self.time = lambda: timeHelper.time()

        MASS = 0.032
        self.planner = firm.planner()
        firm.plan_init(self.planner, MASS)
        self.planner.home = arr2vec(initialPosition)

    # online-planning trajectories
    def takeoff(self, targetHeight, duration):
        firm.plan_takeoff(self.planner,
            self._vposition(), self.yaw(), targetHeight, duration, self.time())

    def land(self, targetHeight, duration):
        firm.plan_land(self.planner,
            self._vposition(), self.yaw(), targetHeight, duration, self.time())

    def hover(self, goal, yaw, duration):
        firm.plan_hover(self.planner, arr2vec(goal), yaw, duration, self.time())

    def goHome(self):
        # TODO
        pass

    # polynomial trajectories
    # TODO: port trajectory object
    # def uploadTrajectory(self, trajectory):
        # # request = UploadTrajectory()
        # # request.polygons = trajectory.polygons
        # self.uploadTrajectoryService(trajectory.polygons)

    def startTrajectory(self):
        firm.plan_start_poly(self.planner, self._vposition(), self.time())

    # TODO: polynomial dictionary exposed from C
    # def startCannedTrajectory(self, trajectory, timescale):

    # ellipse trajectories
    def setEllipse(self, center, major, minor, period):
        e = self.planner.ellipse
        e.center = arr2vec(center)
        e.major = arr2vec(major)
        e.minor = arr2vec(minor)
        e.period = period

    def startEllipse(self):
        firm.plan_start_ellipse(self.planner, self.time())

    # interactive trajectories
    def avoidTarget(self, home, maxDisplacement, maxSpeed):
        firm.plan_start_avoid_target(self.planner,
            arr2vec(home), maxDisplacement, maxSpeed, self.time())

    def updateTarget(self, target):
        firm.plan_update_avoid_target(self.planner, arr2vec(target), self.time())

    # query state
    def position(self):
        pos = self._vposition()
        return np.array([pos.x, pos.y, pos.z])

    def yaw(self):
        ev = firm.plan_current_goal(self.planner, self.time())
        return ev.yaw


    # "private" methods
    def _vposition(self):
        # TODO this should be implemented in C
        if self.planner.state == firm.TRAJECTORY_STATE_IDLE:
            return self.planner.home
        else:
            ev = firm.plan_current_goal(self.planner, self.time())
            # not totally sure why, but if we don't do this, we don't actually return by value
            return firm.mkvec(ev.pos.x, ev.pos.y, ev.pos.z)


class CrazyflieServer:
    def __init__(self, timeHelper):
        with open("../launch/crazyflies.yaml", 'r') as ymlfile:
            cfg = yaml.load(ymlfile)

        def yaml2cf(cfnode):
            id = str(cfnode["id"])
            pos = cfnode["initialPosition"]
            return Crazyflie(id, pos, timeHelper)

        self.crazyflies = [yaml2cf(cfnode) for cfnode in cfg["crazyflies"]]
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
