#!/usr/bin/env python3

"""
Crazyflie Software-In-The-Loop Wrapper that uses the firmware Python bindings


    2022 - Wolfgang Hönig (TU Berlin)
"""
from __future__ import annotations

import numpy as np

import cffirmware as firm


class CrazyflieState:
    def __init__(self, fwstate):
        self.pos = np.array([fwstate.pos.x, fwstate.pos.y, fwstate.pos.z])
        self.vel = np.array([fwstate.vel.x, fwstate.vel.y, fwstate.vel.z])
        self.acc = np.array([fwstate.acc.x, fwstate.acc.y, fwstate.acc.z])
        self.yaw = fwstate.yaw
        self.omega = np.array([fwstate.omega.x, fwstate.omega.y, fwstate.omega.z])

    def __repr__(self) -> str:
        return "{}".format(self.pos)

class TrajectoryPolynomialPiece:
    def __init__(self, poly_x, poly_y, poly_z, poly_yaw, duration):
        self.poly_x = poly_x
        self.poly_y = poly_y
        self.poly_z = poly_z
        self.poly_yaw = poly_yaw
        self.duration = duration


class CrazyflieSIL:

    # Flight modes.
    MODE_IDLE = 0
    MODE_HIGH_POLY = 1
    MODE_LOW_FULLSTATE = 2
    MODE_LOW_POSITION = 3
    MODE_LOW_VELOCITY = 4


    def __init__(self, name, initialPosition, time_func):

        # Core.
        self.name = name
        self.groupMask = 0
        self.initialPosition = np.array(initialPosition)
        self.time_func = time_func

        # Commander.
        self.mode = CrazyflieSIL.MODE_IDLE
        self.planner = firm.planner()
        firm.plan_init(self.planner)
        self.trajectories = dict()
        self.setState = firm.traj_eval_zero()

        # State.
        self.state = firm.traj_eval_zero()
        self.state.pos = firm.mkvec(*initialPosition)
        self.state.vel = firm.vzero()
        self.state.acc = firm.vzero()
        self.state.yaw = 0.0
        self.state.omega = firm.vzero()


    def setGroupMask(self, groupMask):
        self.groupMask = groupMask

    def takeoff(self, targetHeight, duration, groupMask = 0):
        if self._isGroup(groupMask):
            self.mode = CrazyflieSIL.MODE_HIGH_POLY
            targetYaw = 0.0
            firm.plan_takeoff(self.planner,
                self.state.pos, self.state.yaw, targetHeight, targetYaw, duration, self.time_func())

    def land(self, targetHeight, duration, groupMask = 0):
        if self._isGroup(groupMask):
            self.mode = CrazyflieSIL.MODE_HIGH_POLY
            targetYaw = 0.0
            firm.plan_land(self.planner,
                self.state.pos, self.state.yaw, targetHeight, targetYaw, duration, self.time_func())

    # def stop(self, groupMask = 0):
    #     if self._isGroup(groupMask):
    #         self.mode = CrazyflieSIL.MODE_IDLE
    #         firm.plan_stop(self.planner)

    def goTo(self, goal, yaw, duration, relative = False, groupMask = 0):
        if self._isGroup(groupMask):
            if self.mode != CrazyflieSIL.MODE_HIGH_POLY:
                # We need to update to the latest firmware that has go_to_from.
                raise ValueError("goTo from low-level modes not yet supported.")
            self.mode = CrazyflieSIL.MODE_HIGH_POLY
            firm.plan_go_to(self.planner, relative, firm.mkvec(*goal), yaw, duration, self.time_func())

    def uploadTrajectory(self, trajectoryId: int, pieceOffset: int, pieces: list[TrajectoryPolynomialPiece]):
        traj = firm.piecewise_traj()
        traj.t_begin = 0
        traj.timescale = 1.0
        traj.shift = firm.mkvec(0, 0, 0)
        traj.n_pieces = len(pieces)
        traj.pieces = firm.poly4d_malloc(traj.n_pieces)
        for i, piece in enumerate(pieces):
            fwpiece = firm.piecewise_get(traj, i)
            fwpiece.duration = piece.duration
            for coef in range(0, 8):
                firm.poly4d_set(fwpiece, 0, coef, piece.poly_x[coef])
                firm.poly4d_set(fwpiece, 1, coef, piece.poly_y[coef])
                firm.poly4d_set(fwpiece, 2, coef, piece.poly_z[coef])
                firm.poly4d_set(fwpiece, 3, coef, piece.poly_yaw[coef])
        self.trajectories[trajectoryId] = traj

    def startTrajectory(self, trajectoryId: int, timescale: float = 1.0, reverse: bool = False, relative: bool = True, groupMask: int = 0):
        if self._isGroup(groupMask):
            self.mode = CrazyflieSIL.MODE_HIGH_POLY
            traj = self.trajectories[trajectoryId]
            traj.t_begin = self.time_func()
            traj.timescale = timescale
            startfrom = self.state.pos
            firm.plan_start_trajectory(self.planner, traj, reverse, relative, startfrom)

    # def notifySetpointsStop(self, remainValidMillisecs=100):
    #     # No-op - the real Crazyflie prioritizes streaming setpoints over
    #     # high-level commands. This tells it to stop doing that. We don't
    #     # simulate this behavior.
    #     pass

    def cmdFullState(self, pos, vel, acc, yaw, omega):
        self.mode = CrazyflieSIL.MODE_LOW_FULLSTATE
        self.setState.pos = firm.mkvec(*pos)
        self.setState.vel = firm.mkvec(*vel)
        self.setState.acc = firm.mkvec(*acc)
        self.setState.yaw = yaw
        self.setState.omega = firm.mkvec(*omega)

    # def cmdPosition(self, pos, yaw = 0):
    #     self.mode = CrazyflieSIL.MODE_LOW_POSITION
    #     self.setState.pos = firm.mkvec(*pos)
    #     self.setState.yaw = yaw
    #     # TODO: should we set vel, acc, omega to zero, or rely on modes to not read them?

    # def cmdVelocityWorld(self, vel, yawRate):
    #     self.mode = CrazyflieSIL.MODE_LOW_VELOCITY
    #     self.setState.vel = firm.mkvec(*vel)
    #     self.setState.omega = firm.mkvec(0.0, 0.0, yawRate)
    #     # TODO: should we set pos, acc, yaw to zero, or rely on modes to not read them?

    # def cmdStop(self):
    #     # TODO: set mode to MODE_IDLE?
    #     pass

    def getSetpoint(self):
        if self.mode == CrazyflieSIL.MODE_HIGH_POLY:
            self.setState = firm.plan_current_goal(self.planner, self.time_func())
        # else:
            # return CrazyflieState(self.setState)
        setState = firm.traj_eval(self.setState)
        if not firm.is_traj_eval_valid(setState):
            return CrazyflieState(self.state)

        if self.mode == CrazyflieSIL.MODE_IDLE:
            return CrazyflieState(self.state)

        self.state = setState
        return CrazyflieState(setState)

    # "private" methods
    def _isGroup(self, groupMask):
        return groupMask == 0 or (self.groupMask & groupMask) > 0