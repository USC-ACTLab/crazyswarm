from __future__ import annotations

from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.time import Time
from ..simtypes import State, Action


import numpy as np
from rowan.calculus import integrate as quat_integrate
from rowan.functions import _promote_vec, _validate_unit, exp, multiply
from rowan import from_matrix, to_matrix, to_euler, from_euler
from scipy import  integrate, linalg
from numpy.polynomial import Polynomial as poly
import sys

class Backend:

    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)
        self.t = 0
        self.dt = 1e-3

        self.uavs = []
        for state in states:
            uav = UavModel(state)
            self.uavs.append(uav)

    def time(self) -> float:
        return self.t

    def step(self, states_desired: list[State], actions: list[Action]) -> list[State]:
        # advance the time
        self.t += self.dt

        next_states = []

        for uav, action in zip(self.uavs, actions):
            uav.step(action)
            next_states.append(uav.state)

        print(states_desired, actions, next_states)
        # publish the current clock
        clock_message = Clock()
        clock_message.clock = Time(seconds=self.time()).to_msg()
        self.clock_publisher.publish(clock_message)

        # pretend we were able to follow desired states perfectly
        return next_states


def skew(w):
    w = w.reshape(3,1)
    w1 = w[0,0]
    w2 = w[1,0]
    w3 = w[2,0]
    return np.array([[0, -w3, w2],[w3, 0, -w1],[-w2, w1, 0]]).reshape((3,3))

class UavModel:
    """initialize an instance of UAV object with the following physical parameters:
    m = 0.034 [kg]  -------------------------------------> Mass of the UAV
    I =   (16.571710 0.830806 0.718277
            0.830806 16.655602 1.800197    -----------------> Moment of Inertia 
            0.718277 1.800197 29.261652)*10^-6 [kg.m^2]"""

    # State is numpy array with the following components:
    # position x, y, z [m; world frame]
    # velocity vx, vy, vz [m/s; world frame]
    # orientation: qw, qx, qy, qz
    # angular velocity: wx, wy, wz [rad/s]

    def __init__(self, state):
        self.dt = 1e-3
        self.m         = 0.034
        # TODO: switch to the full matrix
        self.I         = np.diag([16.571710e-6, 16.655602e-6, 29.261652e-6])
        self.invI      = linalg.inv(self.I)
        self.d         = 0.046
        self.cft       = 0.006
        arm           = 0.707106781*self.d
        self.invAll = np.array([
            [0.25, -(0.25 / arm), -(0.25 / arm), -(0.25 / self.cft)], #M2
            [0.25, -(0.25 / arm),  (0.25 / arm),  (0.25 / self.cft)], #M3
            [0.25,  (0.25 / arm),  (0.25 / arm), -(0.25 / self.cft)], #M4
            [0.25,  (0.25 / arm), -(0.25 / arm),  (0.25 / self.cft)],  #M1
        ])     
        self.ctrlAll   = linalg.inv(self.invAll)
        self.grav     = np.array([0,0,-self.m*9.81])
            ### State initialized with the Initial values ###
            ### state = [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, wx, wy, wz]
        self.state = state
        
    def getNextAngularState(self, curr_w, curr_q, tau):
        wdot  = self.invI @ (tau - skew(curr_w) @ self.I @ curr_w)
        wNext = wdot * self.dt + curr_w
        qNext = self.integrate_quat(curr_q, curr_w, self.dt)
        return qNext, wNext
        
    def integrate_quat(self, q, wb, dt):
        return multiply(q, exp(_promote_vec(wb * dt / 2))) 

    def getNextLinearState(self, curr_vel, curr_position, q ,fz):
        R_IB = to_matrix(q)
        a =  (1/self.m) * (self.grav + R_IB @ np.array([0,0,fz]))
        velNext = a * self.dt + curr_vel
        posNext = curr_vel * self.dt + curr_position
        return posNext, velNext

    def step(self, action):
        """this method generates the 6D states evolution for the UAV given for each time step:
            the control input: f_th = [f1, f2, f3, f4] for the current step"""

        # convert RPM -> Force
        def rpm_to_force(rpm):
            # if rpm < 1000:
                # return 0
            # polyfit using Tobias' data
            p = [2.55077341e-08, -4.92422570e-05, -1.51910248e-01]
            force_in_grams = np.polyval(p, rpm)
            force_in_newton = force_in_grams * 9.81 / 1000.0
            return np.maximum(force_in_newton, 0)

        # force = rpm_to_force(action.rpm)
        force = action.rpm
        print("force ", force)

        # # while motors are off, we assume we are on the ground and don't update the state
        # if np.all(force == 0):
        #     return self.state


        # compute total trust
        eta = self.ctrlAll @ force


        # Note: we assume here that our control is forces
        arm_length = 0.046 # m
        arm = 0.707106781 * arm_length
        t2t = 0.006 # thrust-to-torque ratio
        B0 = np.array([
            [1, 1, 1, 1],
            [-arm, -arm, arm, arm],
            [-arm, arm, arm, -arm],
            [-t2t, t2t, -t2t, t2t]
            ])
        eta = B0 @ force

        print("eta ", eta)


        fz = eta[0]
        tau_i = eta[1:]

        curr_pos  = self.state.pos  # position: x,y,z
        curr_vel  = self.state.vel  # linear velocity: xdot, ydot, zdot
        curr_q    = self.state.quat # quaternions: [qw, qx, qy, qz]
        curr_w    = self.state.omega  # angular velocity: wx, wy, wz
        
        posNext, velNext = self.getNextLinearState(curr_vel, curr_pos, curr_q, fz)
        qNext, wNext     = self.getNextAngularState(curr_w, curr_q, tau_i)
        
        self.state.pos  = posNext  # position: x,y,z
        self.state.vel  = velNext  # linear velocity: xdot, ydot, zdot
        self.state.quat = qNext# quaternions: [qw, qx, qy, qz]
        self.state.omega = wNext # angular velocity: wx, wy, wz

        # if we fall below the ground, set velocities to 0
        if self.state.pos[2] < 0:
            self.state.pos[2] = 0
            self.state.vel = [0,0,0]
            self.state.omega = [0,0,0]
    
        return self.state

