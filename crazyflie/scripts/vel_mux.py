import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import time

TIME_STOP_SEND_CMD = 0.1

class VelMux(Node):
    def __init__(self):
        super().__init__('vel_mux')
        self.subscription_hp = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.msg_cmd_vel = Twist
        self.received_first_cmd_vel = False

    def cmd_vel_callback(self, msg):
        self.msg = msg
        if self.received_first_cmd_vel is False:
            self.received_first_cmd_vel = True

def main(args=None):
    rclpy.init(args=args)

    vel_mux = VelMux()

    rclpy.spin(vel_mux)

    vel_mux.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()