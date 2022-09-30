#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from crazyflie_interfaces.srv import Takeoff, Land
from crazyflie_interfaces.msg import Hover
import time

HEIGHT = 0.3

class VelMux(Node):
    def __init__(self):
        super().__init__('vel_mux')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.msg_cmd_vel = Twist()
        self.received_first_cmd_vel = False
        prefix = '/cf1'
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.take_off_client = self.create_client(Takeoff, prefix + '/takeoff')
        self.publisher_hover = self.create_publisher(Hover, prefix + '/cmd_hover', 10)
        self.land_client = self.create_client(Land, prefix + '/land')
        self.cf_has_taken_off = False


    def cmd_vel_callback(self, msg):
        self.msg_cmd_vel = msg
        if self.received_first_cmd_vel is False:
            self.received_first_cmd_vel = True

    def timer_callback(self):
        if self.received_first_cmd_vel and self.cf_has_taken_off is False:
            print('take off!')
            req = Takeoff.Request()
            req.height = HEIGHT
            req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
            self.future = self.take_off_client.call_async(req)
            self.cf_has_taken_off = True
            time.sleep(2.0)

        print(self.msg_cmd_vel)
        
        if self.received_first_cmd_vel and self.cf_has_taken_off:
            if self.msg_cmd_vel.linear.z >= 0:
                print('send velocity commands')
                msg = Hover()
                msg.vx = self.msg_cmd_vel.linear.x
                msg.vy = self.msg_cmd_vel.linear.y
                msg.yaw_rate = self.msg_cmd_vel.angular.z
                msg.z_distance = HEIGHT
                self.publisher_hover.publish(msg)
            else:
                req = Land.Request()
                req.height = 0.1
                req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
                self.future = self.land_client.call_async(req)
                self.cf_has_taken_off = False
                self.received_first_cmd_vel = False


def main(args=None):
    rclpy.init(args=args)

    vel_mux = VelMux()

    rclpy.spin(vel_mux)

    vel_mux.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()