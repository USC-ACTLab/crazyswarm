#!/usr/bin/env python

import yaml
import math
import numpy as np

from .cfsim import cffirmware as firm

# main class of simulation.
# crazyflies keep reference to this object to ask what time it is.
# also does the plotting.
#
class TimeHelper:
    def __init__(self, vis, dt, writecsv, disturbanceSize):
        if vis == "mpl":
            from .visualizer import visMatplotlib
            self.visualizer = visMatplotlib.VisMatplotlib()
        elif vis == "vispy":
            from .visualizer import visVispy
            self.visualizer = visVispy.VisVispy()
        elif vis == "null":
            from .visualizer import visNull
            self.visualizer = visNull.VisNull()
        else:
            raise Exception("Unknown visualization backend: {}".format(vis))
        self.t = 0.0
        self.dt = dt
        # Since our integration/animation ticks are always the fixed duration
        # dt, any call to sleep() with a non-multiple of dt will have some
        # "leftover" time. Keep track of it here and add extra ticks in future.
        self.sleepResidual = 0.0
        self.crazyflies = []
        self.disturbanceSize = disturbanceSize
        if writecsv:
            from . import output
            self.output = output.Output()
        else:
            self.output = None

    def time(self):
        return self.t

    def step(self, duration):
        self.t += duration
        for cf in self.crazyflies:
            cf.integrate(duration, self.disturbanceSize)

    # should be called "animate" or something
    # but called "sleep" for source-compatibility with real-robot scripts
    def sleep(self, duration):
        # operator // has unexpected (wrong ?) behavior for this calculation.
        ticks = math.floor((duration + self.sleepResidual) / self.dt)
        self.sleepResidual += duration - self.dt * ticks
        assert 0.0 <= self.sleepResidual < self.dt

        for _ in range(int(ticks)):
            self.visualizer.update(self.t, self.crazyflies)
            if self.output:
                self.output.update(self.t, self.crazyflies)
            self.step(self.dt)

    # Mock for abstraction of rospy.Rate.sleep().
    def sleepForRate(self, rate):
        # TODO: account for rendering time, or is it worth the complexity?
        self.sleep(1.0 / rate)

    # Mock for abstraction of rospy.is_shutdown().
    def isShutdown(self):
        return False

    def addObserver(self, observer):
        self.observers.append(observer)


class Crazyflie:

    # Flight modes.
    MODE_IDLE = 0
    MODE_HIGH_POLY = 1
    MODE_LOW_FULLSTATE = 2
    MODE_LOW_POSITION = 3
    MODE_LOW_VELOCITY = 4


    def __init__(self, id, initialPosition, timeHelper):

        # Core.
        self.id = id
        self.groupMask = 0
        self.initialPosition = np.array(initialPosition)
        self.time = lambda: timeHelper.time()

        # Commander.
        self.mode = Crazyflie.MODE_IDLE
        self.planner = firm.planner()
        firm.plan_init(self.planner)
        self.trajectories = dict()
        self.setState = firm.traj_eval()

        # State. Public np.array-returning getters below for physics state.
        self.state = firm.traj_eval()
        self.state.pos = firm.mkvec(*initialPosition)
        self.state.vel = firm.vzero()
        self.state.acc = firm.vzero()
        self.state.yaw = 0.0
        self.state.omega = firm.vzero()
        self.ledRGB = (0.5, 0.5, 1)

    def setGroupMask(self, groupMask):
        self.groupMask = groupMask

    def takeoff(self, targetHeight, duration, groupMask = 0):
        if self._isGroup(groupMask):
            self.mode = Crazyflie.MODE_HIGH_POLY
            firm.plan_takeoff(self.planner,
                self.state.pos, self.state.yaw, targetHeight, duration, self.time())

    def land(self, targetHeight, duration, groupMask = 0):
        if self._isGroup(groupMask):
            self.mode = Crazyflie.MODE_HIGH_POLY
            firm.plan_land(self.planner,
                self.state.pos, self.state.yaw, targetHeight, duration, self.time())

    def stop(self, groupMask = 0):
        if self._isGroup(groupMask):
            self.mode = Crazyflie.MODE_IDLE
            firm.plan_stop(self.planner)

    def goTo(self, goal, yaw, duration, relative = False, groupMask = 0):
        if self._isGroup(groupMask):
            if self.mode != Crazyflie.MODE_HIGH_POLY:
                # We need to update to the latest firmware that has go_to_from.
                raise ValueError("goTo from low-level modes not yet supported.")
            self.mode = Crazyflie.MODE_HIGH_POLY
            firm.plan_go_to(self.planner, relative, firm.mkvec(*goal), yaw, duration, self.time())

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
            self.mode = Crazyflie.MODE_HIGH_POLY
            traj = self.trajectories[trajectoryId]
            traj.t_begin = self.time()
            traj.timescale = timescale
            if relative:
                traj.shift = firm.vzero()
                if reverse:
                    traj_init = firm.piecewise_eval_reversed(traj, traj.t_begin)
                else:
                    traj_init = firm.piecewise_eval(traj, traj.t_begin)
                traj.shift = firm.vsub(self.state.pos, traj_init.pos)
            else:
                traj.shift = firm.vzero()
            firm.plan_start_trajectory(self.planner, traj, reverse)

    def position(self):
        return np.array(self.state.pos)

    def getParam(self, name):
        print("WARNING: getParam not implemented in simulation!")

    def setParam(self, name, value):
        print("WARNING: setParam not implemented in simulation!")

    def setParams(self, params):
        print("WARNING: setParams not implemented in simulation!")

    # - this is a part of the param system on the real crazyflie,
    #   but we implement it in simulation too for debugging
    # - is a blocking command on real CFs, so may cause stability problems
    def setLEDColor(self, r, g, b):
        self.ledRGB = (r, g, b)

    # simulation only functions
    def yaw(self):
        return float(self.state.yaw)
    
    def velocity(self):
        return np.array(self.state.vel)

    def acceleration(self):
        return np.array(self.state.acc)

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

    def cmdFullState(self, pos, vel, acc, yaw, omega):
        self.mode = Crazyflie.MODE_LOW_FULLSTATE
        self.setState.pos = firm.mkvec(*pos)
        self.setState.vel = firm.mkvec(*vel)
        self.setState.acc = firm.mkvec(*acc)
        self.setState.yaw = yaw
        self.setState.omega = firm.mkvec(*omega)

    def cmdPosition(self, pos, yaw = 0):
        self.mode = Crazyflie.MODE_LOW_POSITION
        self.setState.pos = firm.mkvec(*pos)
        self.setState.yaw = yaw
        # TODO: should we set vel, acc, omega to zero, or rely on modes to not read them?

    def cmdVelocityWorld(self, vel, yawRate):
        self.mode = Crazyflie.MODE_LOW_VELOCITY
        self.setState.vel = firm.mkvec(*vel)
        self.setState.omega = firm.mkvec(0.0, 0.0, yawRate)
        # TODO: should we set pos, acc, yaw to zero, or rely on modes to not read them?

    def cmdStop(self):
        # TODO: set mode to MODE_IDLE?
        pass

    def integrate(self, time, disturbanceSize):

        if self.mode == Crazyflie.MODE_IDLE:
            pass

        elif self.mode == Crazyflie.MODE_HIGH_POLY:
            self.state = firm.plan_current_goal(self.planner, self.time())

        elif self.mode == Crazyflie.MODE_LOW_FULLSTATE:
            self.state = self.setState

        elif self.mode == Crazyflie.MODE_LOW_POSITION:
            # Simple finite difference velocity approxmations.
            velocity = firm.vdiv(firm.vsub(self.setState.pos, self.state.pos), time)
            yawRate = (self.setState.yaw - self.state.yaw) / time
            self.state.pos = self.setState.pos
            self.state.vel = velocity
            self.state.acc = firm.vzero()  # TODO: 2nd-order finite difference? Probably useless.
            self.state.yaw = self.setState.yaw
            self.state.omega = firm.mkvec(0.0, 0.0, yawRate)

        elif self.mode == Crazyflie.MODE_LOW_VELOCITY:
            # Simple Euler integration.
            disturbance = firm.mkvec(*(disturbanceSize * np.random.normal(size=3)))
            velocity = firm.vadd(self.setState.vel, disturbance)
            self.state.pos = firm.vadd(self.state.pos, firm.vscl(time, velocity))
            self.state.vel = velocity
            self.state.acc = firm.vzero()  # TODO: could compute with finite difference
            self.state.yaw += time * self.setState.omega.z
            self.state.omega = self.setState.omega

        else:
            raise ValueError("Unknown flight mode.")


    # "private" methods
    def _isGroup(self, groupMask):
        return groupMask == 0 or (self.groupMask & groupMask) > 0


class CrazyflieServer:
    def __init__(self, timeHelper, crazyflies_yaml="../launch/crazyflies.yaml"):
        """Initialize the server.

        Args:
            timeHelper (TimeHelper): TimeHelper instance.
            crazyflies_yaml (str): If ends in ".yaml", interpret as a path and load
                from file. Otherwise, interpret as YAML string and parse
                directly from string.
        """
        if crazyflies_yaml.endswith(".yaml"):
            with open(crazyflies_yaml, 'r') as ymlfile:
                cfg = yaml.safe_load(ymlfile)
        else:
            cfg = yaml.safe_load(crazyflies_yaml)

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
        print("WARNING: emergency not implemented in simulation!")

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
            crazyflie.goTo(goal, yaw, duration, relative=True, groupMask=groupMask)

    def startTrajectory(self, trajectoryId, timescale = 1.0, reverse = False, relative = True, groupMask = 0):
        for crazyflie in self.crazyflies:
            crazyflie.startTrajectory(trajectoryId, timescale, reverse, relative, groupMask)

    def setParam(self, name, value):
        print("WARNING: setParam not implemented in simulation!")
