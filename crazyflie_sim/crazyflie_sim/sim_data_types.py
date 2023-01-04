import numpy as np

class State:
    """Class that stores the state of a UAV as used in the simulator interface"""
    def __init__(self, pos = np.zeros(3), vel = np.zeros(3), quat = np.array([1,0,0,0]), omega = np.zeros(3)):
        # internally use one numpy array
        self._state = np.empty(13)
        self.pos = pos
        self.vel = vel
        self.quat = quat
        self.omega = omega

    @property
    def pos(self):
        """position [m; world frame]"""
        return self._state[0:3]

    @pos.setter
    def pos(self, value):
        self._state[0:3] = value

    @property
    def vel(self):
        """velocity [m/s; world frame]"""
        return self._state[3:6]

    @vel.setter
    def vel(self, value):
        self._state[3:6] = value

    @property
    def quat(self):
        """quaternion [qw, qx, qy, qz; body -> world]"""
        return self._state[6:10]

    @quat.setter
    def quat(self, value):
        self._state[6:10] = value

    @property
    def omega(self):
        """angular velocity [rad/s; body frame]"""
        return self._state[10:13]

    @omega.setter
    def omega(self, value):
        self._state[10:13] = value

    def __repr__(self) -> str:
        return "State pos={}, vel={}, quat={}, omega={}".format(self.pos, self.vel, self.quat, self.omega)


class Action:
    """Class that stores the action of a UAV as used in the simulator interface"""
    def __init__(self, rpm):
        # internally use one numpy array
        self._action = np.empty(4)
        self.rpm = rpm

    @property
    def rpm(self):
        """rotation per second [rpm]"""
        return self._action

    @rpm.setter
    def rpm(self, value):
        self._action = value

    def __repr__(self) -> str:
        return "Action rpm={}".format(self.rpm)
