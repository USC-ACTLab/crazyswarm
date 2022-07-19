import rclpy
from rclpy.node import Node

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie

import math

from math import pi

URI = 'radio://0/40/2M/E7E7E7E703'

class CrazyflieServer(Node):

    def __init__(self, link_uri):
        super().__init__('crazyflie_server')

        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.open_link(link_uri)

    def _connected(self, link_uri):
        self.get_logger().info('Connected!')

    def _disconnected(self, link_uri):
        self.get_logger().info('Disconnected')

    def _connection_failed(self, link_uri, msg):
        self.get_logger().info('Connection Failed')


def main(args=None):

    cflib.crtp.init_drivers()
    rclpy.init(args=args)
    crazyflie_server = CrazyflieServer(URI)

    rclpy.spin(crazyflie_server)

    crazyflie_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
