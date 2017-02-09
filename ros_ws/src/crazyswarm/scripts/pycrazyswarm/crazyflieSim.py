#!/usr/bin/env python

import yaml
import math
import numpy as np

import cfsim.cffirmware as firm

# "canned trajectory" "enum" from packetdef.h
TRAJECTORY_FIGURE8 = firm.TRAJECTORY_FIGURE8

# main class of simulation.
# crazyflies keep reference to this object to ask what time it is.
# also does the plotting.
#
class TimeHelper:
    def __init__(self, vis, dt, writecsv):
        if vis == "mpl":
            import visualizer.visMatplotlib
            self.visualizer = visualizer.visMatplotlib.VisMatplotlib()
        elif vis == "vispy":
            import visualizer.visVispy
            self.visualizer = visualizer.visVispy.VisVispy()
        else:
            raise Exception("Unknown visualization backend: {}".format(vis))
        self.t = 0.0
        self.dt = dt
        self.crazyflies = []
        if writecsv:
            import output
            self.output = output.Output()
        else:
            self.output = None

    def time(self):
        return self.t

    def step(self, duration):
        self.t += duration

    # should be called "animate" or something
    # but called "sleep" for source-compatibility with real-robot scripts
    def sleep(self, duration):
        for t in np.arange(self.t, self.t + duration, self.dt):
            self.visualizer.update(t, self.crazyflies)
            if self.output:
                self.output.update(t, self.crazyflies)
            self.step(self.dt)

    def nextPhase(self):
        if self.output:
            self.output.nextPhase()

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
        self.planner.lastKnownPosition = arr2vec(initialPosition)
        self.group = 0

    def setGroup(self, group):
        self.group = group

    def _isGroup(self, group):
        return group == 0 or self.group == group

    # online-planning trajectories
    def takeoff(self, targetHeight, duration, group = 0):
        if self._isGroup(group):
            firm.plan_takeoff(self.planner,
                self._vposition(), self.yaw(), targetHeight, duration, self.time())

    def land(self, targetHeight, duration, group = 0):
        if self._isGroup(group):
            firm.plan_land(self.planner,
                self._vposition(), self.yaw(), targetHeight, duration, self.time())

    def hover(self, goal, yaw, duration):
        firm.plan_hover(self.planner, arr2vec(goal), yaw, duration, self.time())

    def goHome(self, group = 0):
        # TODO
        pass

    # polynomial trajectories
    # input can be generated from piecewise.loadcsv().
    def uploadTrajectory(self, trajectory):
        self.user_traj = trajectory

    def startTrajectory(self, group = 0):
        if self._isGroup(group):
            firm.plan_set_ppback(self.planner, self.user_traj)
            firm.plan_start_poly(self.planner, self._vposition(), self.time(), False)

    def startTrajectoryReversed(self, group = 0):
        if self._isGroup(group):
            firm.plan_set_ppback(self.planner, self.user_traj)
            firm.plan_start_poly(self.planner, self._vposition(), self.time(), True)

    def startCannedTrajectory(self, trajectory, timescale, group = 0):
        if self._isGroup(group):
            firm.plan_start_canned_trajectory(self.planner,
                trajectory, timescale, self._vposition(), self.time())

    # ellipse trajectories
    def setEllipse(self, center, major, minor, period):
        e = self.planner.ellipse
        e.center = arr2vec(center)
        e.major = arr2vec(major)
        e.minor = arr2vec(minor)
        e.period = period

    def startEllipse(self, group = 0):
        if self._isGroup(group):
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

    def acceleration(self):
        if self.planner.state == firm.TRAJECTORY_STATE_IDLE:
            return np.array([0, 0, 0])
        else:
            ev = firm.plan_current_goal(self.planner, self.time())
            return np.array([ev.acc.x, ev.acc.y, ev.acc.z])

    def rpy(self):
        acc = self.acceleration()
        yaw = self.yaw()
        norm = np.linalg.norm(acc)
        # print(acc)
        if norm < 1e-6:
            return (0.0, 0.0, yaw)
        else:
            thrust = acc + np.array([0, 0, 9.81])
            z_body = thrust / np.linalg.norm(thrust)
            x_world = np.array([math.cos(yaw), math.sin(yaw), 0])
            y_body = np.cross(z_body, x_world)
            x_body = np.cross(y_body, z_body)
            pitch = math.asin(-x_body[2])
            roll = math.atan2(y_body[2], z_body[2])
            return (roll, pitch, yaw)

    # "private" methods
    def _vposition(self):
        # TODO this should be implemented in C
        if self.planner.state == firm.TRAJECTORY_STATE_IDLE:
            return self.planner.lastKnownPosition
        else:
            ev = firm.plan_current_goal(self.planner, self.time())
            self.planner.lastKnownPosition = firm.mkvec(ev.pos.x, ev.pos.y, ev.pos.z)
            # not totally sure why, but if we don't do this, we don't actually return by value
            return firm.mkvec(ev.pos.x, ev.pos.y, ev.pos.z)


class CrazyflieServer:
    def __init__(self, timeHelper):
        with open("../launch/crazyflies.yaml", 'r') as ymlfile:
            cfg = yaml.load(ymlfile)

        self.crazyflies = []
        self.crazyfliesById = dict()
        for crazyflie in cfg["crazyflies"]:
            id = int(crazyflie["id"])
            initialPosition = crazyflie["initialPosition"]
            cf = Crazyflie(id, initialPosition, timeHelper)
            self.crazyflies.append(cf)
            self.crazyfliesById[id] = cf

        self.timeHelper = timeHelper
        self.timeHelper.crazyflies = self.crazyflies

    def emergency(self):
        # TODO
        pass

    def takeoff(self, targetHeight, duration, group = 0):
        for crazyflie in self.crazyflies:
            crazyflie.takeoff(targetHeight, duration, group)

    def land(self, targetHeight, duration, group = 0):
        for crazyflie in self.crazyflies:
            crazyflie.land(targetHeight, duration, group)

    def startTrajectory(self, group = 0):
        for crazyflie in self.crazyflies:
            crazyflie.startTrajectory(group)

    def startTrajectoryReversed(self, group = 0):
        for crazyflie in self.crazyflies:
            crazyflie.startTrajectoryReversed(group)

    def startEllipse(self, group = 0):
        for crazyflie in self.crazyflies:
            crazyflie.startEllipse(group)

    def startCannedTrajectory(self, trajectory, timescale, group = 0):
        for crazyflie in self.crazyflies:
            crazyflie.startCannedTrajectory(trajectory, timescale, group)

    def goHome(self, group = 0):
        for crazyflie in self.crazyflies:
            crazyflie.goHome(group)
