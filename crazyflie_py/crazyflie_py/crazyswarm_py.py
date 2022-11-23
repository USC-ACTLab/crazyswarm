import os
import rclpy

from . import genericJoystick
from .crazyflie import TimeHelper, CrazyflieServer


class Crazyswarm:
    def __init__(self, crazyflies_yaml=None):
        if crazyflies_yaml is None:
            from ament_index_python.packages import get_package_share_directory
            crazyflies_yaml = os.path.join(
                get_package_share_directory('crazyflie'),
                'config',
                'crazyflies.yaml')
        if crazyflies_yaml.endswith(".yaml"):
            crazyflies_yaml = open(crazyflies_yaml, 'r').read()

        rclpy.init()
        self.allcfs = CrazyflieServer(crazyflies_yaml)
        self.timeHelper = TimeHelper(self.allcfs)

        self.input = genericJoystick.Joystick(self.timeHelper)
