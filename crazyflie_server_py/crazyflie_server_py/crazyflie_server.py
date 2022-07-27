from argparse import Namespace
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie

from crazyswarm2_interfaces.srv import Takeoff, Land, GoTo

import math

from math import pi


class CrazyflieServer(Node):

    def __init__(self):
        super().__init__("crazyflie_server")
        self._cf = Crazyflie(rw_cache="./cache")

        # Get Parameter values
        self.declare_parameters(namespace="",
            parameters=[
                ("uri", "radio://0/80/2M/E7E7E7E7E7")
            ])
        link_uri = self.get_parameter("uri").get_parameter_value().string_value
        print("Trying to connect to " + link_uri)

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.open_link(link_uri)

    def _connected(self, link_uri):
        self.get_logger().info("Connected!")
        self.create_service(Takeoff, "/takeoff", self._takeoff_callback )
        self.create_service(Land, "/land", self._land_callback )
        self.create_service(GoTo, "/go_to", self._go_to_callback )
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_changed, 10)

    def _disconnected(self, link_uri):
        self.get_logger().info("Disconnected")
        self.destroy_node()

    def _connection_failed(self, link_uri, msg):
        self.get_logger().info("Connection Failed")
        self.destroy_node()

    def _takeoff_callback(self, request, response):
        duration = float(request.duration.sec) + float(request.duration.nanosec / 1e9)
        self.get_logger().info(f"takeoff(height={request.height} m," +
            f"duration={duration} s," +
            f"group_mask={request.group_mask})")
        self._cf.high_level_commander.takeoff(1.0, 2.0)
        return response
    
    def _land_callback(self, request, response):
        duration = float(request.duration.sec) + float(request.duration.nanosec / 1e9)
        self.get_logger().info(f"land(height={request.height} m," +
            f"duration={duration} s," +
            f"group_mask={request.group_mask})")
        self._cf.high_level_commander.land(request.height,
            duration, group_mask=request.group_mask)
        return response

    def _go_to_callback(self, request, response):
        duration = float(request.duration.sec) + float(request.duration.nanosec / 1e9)
        self.get_logger().info("go_to(position=%f,%f,%f m, yaw=%f rad, duration=%f s, relative=%d, group_mask=%d)" %
            (request.goal.x, request.goal.y, request.goal.z, request.yaw,
            duration, request.relative, request.group_mask))
        self._cf.high_level_commander.go_to(
            request.goal.x, request.goal.y, request.goal.z, request.yaw,
            duration, relative=request.relative, group_mask=request.group_mask)
        return response

    def _cmd_velocity_changed(self, msg):
        roll = msg.linear.y
        pitch = - msg.linear.x
        yawrate = msg.angular.z
        thrust = int(min(max(msg.linear.z, 0,0), 60000))
        self._cf.sendSetpoint(roll, pitch, yawrate, thrust)


def main(args=None):

    cflib.crtp.init_drivers()
    rclpy.init(args=args)
    crazyflie_server = CrazyflieServer()

    rclpy.spin(crazyflie_server)

    crazyflie_server.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
