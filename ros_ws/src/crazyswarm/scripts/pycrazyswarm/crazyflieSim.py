#!/usr/bin/env python

import yaml
import math
import numpy as np

import cfsim.cffirmware as firm

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

        self.planner = firm.planner()
        firm.plan_init(self.planner)
        self.planner.lastKnownPosition = arr2vec(initialPosition)
        self.groupMask = 0
        self.trajectories = dict()

    def setGroupMask(self, groupMask):
        self.groupMask = groupMask

    def takeoff(self, targetHeight, duration, groupMask = 0):
        if self._isGroup(groupMask):
            firm.plan_takeoff(self.planner,
                self._vposition(), self.yaw(), targetHeight, duration, self.time())

    def land(self, targetHeight, duration, groupMask = 0):
        if self._isGroup(groupMask):
            firm.plan_land(self.planner,
                self._vposition(), self.yaw(), targetHeight, duration, self.time())

    def stop(self, groupMask = 0):
        if self._isGroup(groupMask):
            firm.plan_stop(self.planner)

    def goTo(self, goal, yaw, duration, relative = False, groupMask = 0):
        if self._isGroup(groupMask):
            firm.plan_go_to(self.planner, relative, arr2vec(goal), yaw, duration, self.time())

    def uploadTrajectory(self, trajectoryId, pieceOffset, trajectory):
        traj = firm.piecewise_traj()
        traj.t_begin = 0
        traj.timescale = 1.0
        traj.shift = firm.mkvec(0, 0, 0)
        traj.n_pieces = len(trajectory.polynomials)
        traj.pieces = firm.malloc_poly4d(len(trajectory.polynomials))
        for i, poly in enumerate(trajectory.polynomials):
            piece = firm.pp_get_piece(traj, i)
            piece.duration = poly.duration
            for coef in range(0, 8):
                firm.poly4d_set(piece, 0, coef, poly.px.p[coef])
                firm.poly4d_set(piece, 1, coef, poly.py.p[coef])
                firm.poly4d_set(piece, 2, coef, poly.pz.p[coef])
                firm.poly4d_set(piece, 3, coef, poly.pyaw.p[coef])
        self.trajectories[trajectoryId] = traj

    def startTrajectory(self, trajectoryId, timescale = 1.0, reverse = False, relative = True, groupMask = 0):
        if self._isGroup(groupMask):
            traj = self.trajectories[trajectoryId]
            traj.t_begin = self.time()
            traj.timescale = timescale
            if relative:
                pos = self._vposition()
                traj.shift = firm.vzero()
                if reverse:
                    traj_init = firm.piecewise_eval_reversed(traj, traj.t_begin)
                else:
                    traj_init = firm.piecewise_eval(traj, traj.t_begin)
                traj.shift = firm.vsub(pos, traj_init.pos)
            else:
                traj.shift = firm.vzero()
            firm.plan_start_trajectory(self.planner, traj, reverse)

    def position(self):
        pos = self._vposition()
        return np.array([pos.x, pos.y, pos.z])

    def getParam(self, name):
        print("WARNING: getParam not implemented in simulation!")

    def setParam(self, name, value):
        print("WARNING: setParam not implemented in simulation!")

    def setParams(self, params):
        print("WARNING: setParams not implemented in simulation!")

    # simulation only functions
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
    def _isGroup(self, groupMask):
        return groupMask == 0 or (self.groupMask & groupMask) > 0

    def _vposition(self):
        # TODO this should be implemented in C
        # print(self.id, self.planner, self.planner.state)
        if self.planner.state == firm.TRAJECTORY_STATE_IDLE:
            return self.planner.lastKnownPosition
        else:
            ev = firm.plan_current_goal(self.planner, self.time())
            self.planner.lastKnownPosition = firm.mkvec(ev.pos.x, ev.pos.y, ev.pos.z)
            # print(self.id, ev.pos.z)
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
        print("WARNING: setParams not implemented in simulation!")

    def takeoff(self, targetHeight, duration, groupMask = 0):
        for crazyflie in self.crazyflies:
            crazyflie.takeoff(targetHeight, duration, groupMask)

    def land(self, targetHeight, duration, groupMask = 0):
        for crazyflie in self.crazyflies:
            crazyflie.land(targetHeight, duration, groupMask)

    def stop(self, groupMask = 0):
        for crazyflie in self.crazyflies:
            crazyflie.stop(groupMask)

    def goTo(self, goal, yaw, duration, groupMask = 0):
        for crazyflie in self.crazyflies:
            crazyflie.goTo(goal, yaw, duration, False, groupMask)

    def startTrajectory(self, trajectoryId, timescale = 1.0, reverse = False, relative = True, groupMask = 0):
        for crazyflie in self.crazyflies:
            crazyflie.startTrajectory(trajectoryId, timescale, reverse, relative, groupMask)
