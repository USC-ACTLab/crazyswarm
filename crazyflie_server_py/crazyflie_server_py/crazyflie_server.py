from argparse import Namespace
import rclpy
from rclpy.node import Node

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie

import math

from math import pi


class CrazyflieServer(Node):

    def __init__(self):
        super().__init__('crazyflie_server')
        self._cf = Crazyflie(rw_cache='./cache')

        #cflist = self.get_parameters('crazyflies')
        #print(cflist)

        self.declare_parameters(namespace='',
            parameters=[
                ('uri', 'radio://0/80/2M/E7E7E7E7E7')
            ])

        link_uri = self.get_parameter('uri').get_parameter_value().string_value
        print('Trying to connect to ' + link_uri)

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.open_link(link_uri)

    def _connected(self, link_uri):
        self.get_logger().info('Connected!')

    def _disconnected(self, link_uri):
        self.get_logger().info('Disconnected')
        self.destroy_node()

    def _connection_failed(self, link_uri, msg):
        self.get_logger().info('Connection Failed')
        self.destroy_node()

def main(args=None):

    cflib.crtp.init_drivers()
    rclpy.init(args=args)
    crazyflie_server = CrazyflieServer()

    rclpy.spin(crazyflie_server)

    crazyflie_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
