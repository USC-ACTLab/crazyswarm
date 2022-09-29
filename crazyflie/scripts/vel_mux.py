import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from crazyflie_interfaces.srv import Takeoff, Land
from crazyflie_interfaces.msg import Hover
import time

HEIGHT = 0.8

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
        prefix = '/cf2'
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.take_off_client = self.create_client(Takeoff, prefix + '/take_off')
        self.publisher_hover = self.create_publisher(Hover, prefix + '/cmd_vel_hover/', 10)
        self.land_client = self.create_client(Takeoff, prefix + '/land')


    def cmd_vel_callback(self, msg):
        self.msg_cmd_vel = msg
        if self.received_first_cmd_vel is False:
            self.received_first_cmd_vel = True

    def timer_callback(self):
        if self.received_first_cmd_vel and self.cf_has_taken_off is False:
            print('take off!')
            req = Takeoff.request()
            req.height = HEIGHT
            req.duration = 2.0
            self.future = self.take_off_client.call_async(req)
            rclpy.spin_until_future_complete(self, self.future)
            self.cf_has_taken_off = True
        
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
                req = Land.request()
                req.height = 0.1
                req.duration = 2.0
                self.future = self.land_client.call_async(req)
                rclpy.spin_until_future_complete(self, self.future)
                self.cf_has_taken_off = False


def main(args=None):
    rclpy.init(args=args)

    vel_mux = VelMux()

    rclpy.spin(vel_mux)

    vel_mux.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()