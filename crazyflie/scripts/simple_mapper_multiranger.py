#!/usr/bin/env python3

""" This simple mapper is loosely based on both the bitcraze cflib point cloud example 
 https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/multiranger/multiranger_pointcloud.py
 and the webots epuck simple mapper example:
 https://github.com/cyberbotics/webots_ros2

 Originally from https://github.com/knmcguire/crazyflie_ros2_experimental/
 """

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

import tf_transformations
import math
import numpy as np
from bresenham import bresenham

GLOBAL_SIZE_X = 20.0
GLOBAL_SIZE_Y = 20.0
MAP_RES = 0.1


class SimpleMapperMultiranger(Node):
    def __init__(self):
        super().__init__('simple_mapper_multiranger')
        self.declare_parameter('robot_prefix', '/cf231')
        robot_prefix = self.get_parameter('robot_prefix').value

        self.odom_subscriber = self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odom_subscribe_callback, 10)
        self.ranges_subscriber = self.create_subscription(
            LaserScan, robot_prefix + '/scan', self.scan_subscribe_callback, 10)
        self.position = [0.0, 0.0, 0.0]
        self.angles = [0.0, 0.0, 0.0]
        self.ranges = [0.0, 0.0, 0.0, 0.0]
        self.range_max = 3.5

        self.tfbr = StaticTransformBroadcaster(self)
        t_map = TransformStamped()
        t_map.header.stamp = self.get_clock().now().to_msg()
        t_map.header.frame_id = 'map'
        t_map.child_frame_id = 'odom'
        t_map.transform.translation.x = 0.0
        t_map.transform.translation.y = 0.0
        t_map.transform.translation.z = 0.0
        self.tfbr.sendTransform(t_map)

        self.position_update = False

        self.map = [-1] * int(GLOBAL_SIZE_X / MAP_RES) * \
            int(GLOBAL_SIZE_Y / MAP_RES)
        self.map_publisher = self.create_publisher(OccupancyGrid, robot_prefix + '/map',
                                                   qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST,))

        self.get_logger().info(f"Simple mapper set for crazyflie " + robot_prefix +
                               f" using the odom and scan topic")

    def odom_subscribe_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles[0] = euler[0]
        self.angles[1] = euler[1]
        self.angles[2] = euler[2]
        self.position_update = True

    def scan_subscribe_callback(self, msg):
        self.ranges = msg.ranges
        self.range_max = msg.range_max
        data = self.rotate_and_create_points()

        points_x = []
        points_y = []
        #
        if self.position_update is False:
            return
        for i in range(len(data)):
            point_x = int((data[i][0] - GLOBAL_SIZE_X / 2.0) / MAP_RES)
            point_y = int((data[i][1] - GLOBAL_SIZE_Y / 2.0) / MAP_RES)
            points_x.append(point_x)
            points_y.append(point_y)
            position_x_map = int(
                (self.position[0] - GLOBAL_SIZE_X / 2.0) / MAP_RES)
            position_y_map = int(
                (self.position[1] - GLOBAL_SIZE_Y / 2.0) / MAP_RES)
            for line_x, line_y in bresenham(position_x_map, position_y_map, point_x, point_y):
                self.map[line_y * int(GLOBAL_SIZE_X / MAP_RES) + line_x] = 0
            self.map[point_y * int(GLOBAL_SIZE_X / MAP_RES) + point_x] = 100

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = MAP_RES
        msg.info.width = int(GLOBAL_SIZE_X / MAP_RES)
        msg.info.height = int(GLOBAL_SIZE_Y / MAP_RES)
        msg.info.origin.position.x = - GLOBAL_SIZE_X / 2.0
        msg.info.origin.position.y = - GLOBAL_SIZE_Y / 2.0
        msg.data = self.map
        self.map_publisher.publish(msg)

    def rotate_and_create_points(self):
        data = []
        o = self.position
        roll = self.angles[0]
        pitch = self.angles[1]
        yaw = self.angles[2]
        r_back = self.ranges[0]
        r_right = self.ranges[1]
        r_front = self.ranges[2]
        r_left = self.ranges[3]

        if (r_left < self.range_max and r_left != 0.0):
            left = [o[0], o[1] + r_left, o[2]]
            data.append(self.rot(roll, pitch, yaw, o, left))

        if (r_right < self.range_max and r_right != 0.0):
            right = [o[0], o[1] - r_right, o[2]]
            data.append(self.rot(roll, pitch, yaw, o, right))

        if (r_front < self.range_max and r_front != 0.0):
            front = [o[0] + r_front, o[1], o[2]]
            data.append(self.rot(roll, pitch, yaw, o, front))

        if (r_back < self.range_max and r_back != 0.0):
            back = [o[0] - r_back, o[1], o[2]]
            data.append(self.rot(roll, pitch, yaw, o, back))

        return data

    def rot(self, roll, pitch, yaw, origin, point):
        cosr = math.cos((roll))
        cosp = math.cos((pitch))
        cosy = math.cos((yaw))

        sinr = math.sin((roll))
        sinp = math.sin((pitch))
        siny = math.sin((yaw))

        roty = np.array([[cosy, -siny, 0],
                        [siny, cosy, 0],
                        [0, 0,    1]])

        rotp = np.array([[cosp, 0, sinp],
                        [0, 1, 0],
                        [-sinp, 0, cosp]])

        rotr = np.array([[1, 0,   0],
                        [0, cosr, -sinr],
                        [0, sinr,  cosr]])

        rotFirst = np.dot(rotr, rotp)

        rot = np.array(np.dot(rotFirst, roty))

        tmp = np.subtract(point, origin)
        tmp2 = np.dot(rot, tmp)
        return np.add(tmp2, origin)


def main(args=None):

    rclpy.init(args=args)
    simple_mapper_multiranger = SimpleMapperMultiranger()
    rclpy.spin(simple_mapper_multiranger)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
