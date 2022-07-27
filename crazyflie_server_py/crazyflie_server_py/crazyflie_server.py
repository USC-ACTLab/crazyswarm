from argparse import Namespace
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Twist

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

from crazyswarm2_interfaces.srv import Takeoff, Land, GoTo

import math
import os
import yaml
from math import pi
from functools import partial

class CrazyflieServer(Node):

    def __init__(self):
        super().__init__("crazyflie_server")
        #self._cf = Crazyflie(rw_cache="./cache")

        crazyflies_yaml = os.path.join(
        get_package_share_directory('crazyswarm2'),
        'config',
        'crazyflies.yaml')

        with open(crazyflies_yaml) as f:
            data = yaml.safe_load(f)
        self.uris = []
        cf_dict = {}
        for crazyflie in data:
            uri = data[crazyflie]['uri']
            self.uris.append(uri)
            cf_dict[uri]= crazyflie

        factory = CachedCfFactory(rw_cache='./cache')

        self.swarm = Swarm(self.uris, factory=factory)
        for link_uri in self.uris:
            self.swarm._cfs[link_uri].cf.connected.add_callback(self._connected)
            self.swarm._cfs[link_uri].cf.disconnected.add_callback(self._disconnected)
            self.swarm._cfs[link_uri].cf.connection_failed.add_callback(self._connection_failed)
        self.swarm.open_links()
        self.create_service(Takeoff, "/takeoff", self._takeoff_callback)
        self.create_service(Land, "/land", self._land_callback )
        self.create_service(GoTo, "/go_to", self._go_to_callback )

        for uri in cf_dict:
            name = cf_dict[uri]
            self.create_service(Takeoff, name + "/takeoff", partial(self._takeoff_callback, uri=uri))
            self.create_service(Land, name + "/land", partial(self._land_callback, uri=uri) )
            self.create_service(GoTo, name + "/go_to", partial(self._go_to_callback, uri=uri)  )
            self.create_subscription(Twist, name + '/cmd_vel', partial(self._cmd_vel_changed, uri=uri), 10)

    def _connected(self, link_uri):
        self.get_logger().info(f" {link_uri} is connected!")

    def _disconnected(self, link_uri):
        self.get_logger().info(f" {link_uri} is disconnected!")

    def _connection_failed(self, link_uri, msg):
        self.get_logger().info(f"{link_uri} connection Failed")
        self.swarm.close_links()

    def _takeoff_callback(self, request, response, uri='all'):
        duration = float(request.duration.sec) + float(request.duration.nanosec / 1e9)
        self.get_logger().info(f"takeoff(height={request.height} m," +
            f"duration={duration} s," +
            f"group_mask={request.group_mask}) {uri}")
        if uri == 'all':
            for link_uri in self.uris:
                self.swarm._cfs[link_uri].cf.high_level_commander.takeoff(request.height, duration)
        else:
            self.swarm._cfs[uri].cf.high_level_commander.takeoff(request.height, duration)

        return response
    
    def _land_callback(self, request, response, uri='all'):
        duration = float(request.duration.sec) + float(request.duration.nanosec / 1e9)
        self.get_logger().info(f"land(height={request.height} m," +
            f"duration={duration} s," +
            f"group_mask={request.group_mask})")
        if uri == 'all':
            for link_uri in self.uris:
                self.swarm._cfs[link_uri].cf.high_level_commander.land(request.height,
                    duration, group_mask=request.group_mask)
        else:
            self.swarm._cfs[uri].cf.high_level_commander.land(request.height,
                duration, group_mask=request.group_mask)
               
        return response

    def _go_to_callback(self, request, response, uri='all'):
        duration = float(request.duration.sec) + float(request.duration.nanosec / 1e9)
        self.get_logger().info("go_to(position=%f,%f,%f m, yaw=%f rad, duration=%f s, relative=%d, group_mask=%d)" %
            (request.goal.x, request.goal.y, request.goal.z, request.yaw,
            duration, request.relative, request.group_mask))
        if uri == 'all':
            for link_uri in self.uris:
                self.swarm._cfs[link_uri].cf.high_level_commander.go_to(
                    request.goal.x, request.goal.y, request.goal.z, request.yaw,
                    duration, relative=request.relative, group_mask=request.group_mask)
        else:
            self.swarm._cfs[uri].cf.high_level_commander.go_to(
                request.goal.x, request.goal.y, request.goal.z, request.yaw,
                duration, relative=request.relative, group_mask=request.group_mask)
        return response

    def _cmd_vel_changed(self, msg, uri=''):
        print(uri)
        roll = msg.linear.y
        pitch = - msg.linear.x
        yawrate = msg.angular.z
        thrust = int(min(max(msg.linear.z, 0,0), 60000))
        self.swarm._cfs[uri].cf.commander.send_setpoint(roll, pitch, yawrate, thrust)


def main(args=None):

    cflib.crtp.init_drivers()
    rclpy.init(args=args)
    crazyflie_server = CrazyflieServer()

    rclpy.spin(crazyflie_server)

    crazyflie_server.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
