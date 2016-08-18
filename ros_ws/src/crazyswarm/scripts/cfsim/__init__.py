#!/usr/bin/env python

import sys
import yaml
import numpy as np
import cffirmware as cfswig

def arr2vec(a):
    return cfswig.mkvec(a[0], a[1], a[2])

class CrazyflieSim:

    def __init__(self, id, initialPosition):
        self.id = id
        self.initialPosition = np.array(initialPosition)

        MASS = 0.032
        self.planner = cfswig.planner()
        cfswig.plan_init(self.planner, MASS)
        self.planner.home = arr2vec(initialPosition)


    # TODO: port trajectory object
    # def uploadTrajectory(self, trajectory):
        # # request = UploadTrajectory()
        # # request.polygons = trajectory.polygons
        # self.uploadTrajectoryService(trajectory.polygons)

    def setEllipse(self, center, major, minor, period):
        e = self.planner.ellipse
        e.center = arr2vec(center)
        e.major = arr2vec(major)
        e.minor = arr2vec(minor)
        e.period = period

    def takeoff(self, targetHeight, duration, t):
        pos = self._vposition(t)
        cfswig.plan_takeoff(self.planner,
            self._vposition(t), self.yaw(t), targetHeight, duration, t)

    def land(self, targetHeight, duration, t):
        cfswig.plan_land(self.planner,
            self._vposition(t), self.yaw(t), targetHeight, duration, t)

    def hover(self, goal, yaw, duration, t):
        cfswig.plan_hover(self.planner, arr2vec(goal), yaw, duration, t)

    def avoidTarget(self, home, maxDisplacement, maxSpeed, t):
        cfswig.plan_start_avoid_target(self.planner,
            arr2vec(home), maxDisplacement, maxSpeed, t)

    def startTrajectory(self, t):
        cfswig.plan_start_poly(self.planner, self._vposition(t), t)

    def startEllipse(self, t):
        cfswig.plan_start_ellipse(self.planner, t)

    # TODO: polynomial dictionary exposed from C
    # def startCannedTrajectory(self, trajectory, timescale, t):

    def goHome(self, t):
        for cf in self.crazyflies:
            cf.goHome(t)

    def position(self, t):
        pos = self._vposition(t)
        return np.array([pos.x, pos.y, pos.z])

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

class CrazyflieServerSim:

    def __init__(self):
        with open("../launch/crazyflies.yaml", 'r') as ymlfile:
            cfg = yaml.load(ymlfile)

        yaml2cf = lambda cf: CrazyflieSim(str(cf["id"]), cf["initialPosition"])
        self.crazyflies = [yaml2cf(crazyflie) for crazyflie in cfg["crazyflies"]]

    def takeoff(self, targetHeight, duration, t):
        for cf in self.crazyflies:
            cf.takeoff(targetHeight, duration, t)

    def land(self, targetHeight, duration, t):
        for cf in self.crazyflies:
            cf.land(targetHeight, duration, t)

    def startTrajectory(self, t):
        for cf in self.crazyflies:
            cf.startTrajectory(t)

    def startEllipse(self, t):
        for cf in self.crazyflies:
            cf.startEllipse(t)

    def startCannedTrajectory(self, trajectory, timescale, t):
        for cf in self.crazyflies:
            cf.startCannedTrajectory(trajectory, timescale, t)

    def goHome(self, t):
        for cf in self.crazyflies:
            cf.goHome(t)
