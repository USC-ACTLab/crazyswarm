#!/usr/bin/env python3

"""
Crazyflie Software-In-The-Loop Wrapper that uses the firmware Python bindings


    2022 - Wolfgang Hönig (TU Berlin)
"""

import numpy as np

import cffirmware as firm


class CrazyflieState:
    def __init__(self, fwstate):
        self.pos = np.array([fwstate.pos.x, fwstate.pos.y, fwstate.pos.z])

    def __repr__(self) -> str:
        return "{}".format(self.pos)


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

        # State. Public np.array-returning getters below for physics state.
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

    # def goTo(self, goal, yaw, duration, relative = False, groupMask = 0):
    #     if self._isGroup(groupMask):
    #         if self.mode != CrazyflieSIL.MODE_HIGH_POLY:
    #             # We need to update to the latest firmware that has go_to_from.
    #             raise ValueError("goTo from low-level modes not yet supported.")
    #         self.mode = CrazyflieSIL.MODE_HIGH_POLY
    #         firm.plan_go_to(self.planner, relative, firm.mkvec(*goal), yaw, duration, self.time_func())

    # def uploadTrajectory(self, trajectoryId, pieceOffset, trajectory):
    #     traj = firm.piecewise_traj()
    #     traj.t_begin = 0
    #     traj.timescale = 1.0
    #     traj.shift = firm.mkvec(0, 0, 0)
    #     traj.n_pieces = len(trajectory.polynomials)
    #     traj.pieces = firm.malloc_poly4d(len(trajectory.polynomials))
    #     for i, poly in enumerate(trajectory.polynomials):
    #         piece = firm.pp_get_piece(traj, i)
    #         piece.duration = poly.duration
    #         for coef in range(0, 8):
    #             firm.poly4d_set(piece, 0, coef, poly.px.p[coef])
    #             firm.poly4d_set(piece, 1, coef, poly.py.p[coef])
    #             firm.poly4d_set(piece, 2, coef, poly.pz.p[coef])
    #             firm.poly4d_set(piece, 3, coef, poly.pyaw.p[coef])
    #     self.trajectories[trajectoryId] = traj

    # def startTrajectory(self, trajectoryId, timescale = 1.0, reverse = False, relative = True, groupMask = 0):
    #     if self._isGroup(groupMask):
    #         self.mode = CrazyflieSIL.MODE_HIGH_POLY
    #         traj = self.trajectories[trajectoryId]
    #         traj.t_begin = self.time_func()
    #         traj.timescale = timescale
    #         if relative:
    #             traj.shift = firm.vzero()
    #             if reverse:
    #                 traj_init = firm.piecewise_eval_reversed(traj, traj.t_begin)
    #             else:
    #                 traj_init = firm.piecewise_eval(traj, traj.t_begin)
    #             traj.shift = self.state.pos - traj_init.pos
    #         else:
    #             traj.shift = firm.vzero()
    #         firm.plan_start_trajectory(self.planner, traj, reverse)

    # def notifySetpointsStop(self, remainValidMillisecs=100):
    #     # No-op - the real Crazyflie prioritizes streaming setpoints over
    #     # high-level commands. This tells it to stop doing that. We don't
    #     # simulate this behavior.
    #     pass

    # def cmdFullState(self, pos, vel, acc, yaw, omega):
    #     self.mode = CrazyflieSIL.MODE_LOW_FULLSTATE
    #     self.setState.pos = firm.mkvec(*pos)
    #     self.setState.vel = firm.mkvec(*vel)
    #     self.setState.acc = firm.mkvec(*acc)
    #     self.setState.yaw = yaw
    #     self.setState.omega = firm.mkvec(*omega)

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