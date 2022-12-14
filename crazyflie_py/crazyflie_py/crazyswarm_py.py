import rclpy

from . import genericJoystick
from .crazyflie import TimeHelper, CrazyflieServer


class Crazyswarm:
    def __init__(self):
        rclpy.init()
        
        self.allcfs = CrazyflieServer()
        self.timeHelper = TimeHelper(self.allcfs)

        self.input = genericJoystick.Joystick(self.timeHelper)
