#!/usr/bin/env python3

"""
A crazyflie server for communicating with several crazyflies
    based on the official crazyflie python library from 
    Bitcraze AB


    2022 - K. N. McGuire (Bitcraze AB)
"""

import rclpy
from rclpy.node import Node
import time

from crazyflie_interfaces.srv import Takeoff, Land, GoTo, RemoveLogging, AddLogging
from crazyflie_interfaces.srv import UploadTrajectory, StartTrajectory, NotifySetpointsStop
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, ParameterType
from crazyflie_interfaces.msg import Hover

from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import UInt8, UInt16, UInt32, Int8, Int16, Int32, Float32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# import tf_transformations
# from tf2_ros import TransformBroadcaster

from functools import partial
from math import degrees, radians, pi, cos, sin

# import BackendRviz from .backend_rviz
from .backend_rviz import BackendRviz
from .crazyflie_sil import CrazyflieSIL


class CrazyflieServer(Node):
    def __init__(self):
        super().__init__(
            "crazyflie_server",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # Turn ROS parameters into a dictionary
        self._ros_parameters = self._param_to_dict(self._parameters)

        # self.uris = []
        # self.cf_dict = {}
        # self.uri_dict = {}
        # self.type_dict = {}
        self.cfs = dict()
        
        self.world_tf_name = "world"
        try:
            self.world_tf_name = self._ros_parameters["world_tf_name"]
        except KeyError:
            pass
        robot_data = self._ros_parameters["robots"]

        # initialize backend
        self.backend = BackendRviz(self)

        # Create easy lookup tables for uri, name and types
        for cfname in robot_data:
            if robot_data[cfname]["enabled"]:
                self.cfs[cfname] = CrazyflieSIL(
                    cfname,
                    robot_data[cfname]["initial_position"],
                    self.backend.time)

        # Create services for the entire swarm and each individual crazyflie
        self.create_service(Empty, "all/emergency", self._emergency_callback)
        self.create_service(Takeoff, "all/takeoff", self._takeoff_callback)
        self.create_service(Land, "all/land", self._land_callback)
        self.create_service(GoTo, "all/go_to", self._go_to_callback)
        self.create_service(StartTrajectory, "all/start_trajectory", self._start_trajectory_callback)

        for name, _ in self.cfs.items():
            self.create_service(
                Empty, name +
                "/emergency", partial(self._emergency_callback, name=name)
            )
            self.create_service(
                Takeoff, name +
                "/takeoff", partial(self._takeoff_callback, name=name)
            )
            self.create_service(
                Land, name + "/land", partial(self._land_callback, name=name)
            )
            self.create_service(
                GoTo, name + "/go_to", partial(self._go_to_callback, name=name)
            )
            self.create_service(
                StartTrajectory, name + "/start_trajectory", partial(self._start_trajectory_callback, name=name)
            )
            self.create_service(
                UploadTrajectory, name + "/upload_trajectory", partial(self._upload_trajectory_callback, name=name) 
            )
            self.create_service(
                NotifySetpointsStop, name + "/notify_setpoints_stop", partial(self._notify_setpoints_stop_callback, name=name) 
            )
            self.create_subscription(
                Twist, name +
                "/cmd_vel_legacy", partial(self._cmd_vel_legacy_changed, name=name), 10
            )
            self.create_subscription(
                Hover, name +
                "/cmd_hover", partial(self._cmd_hover_changed, name=name), 10
            )

        # Initialize backend
        self.backend.init([name for name, _ in self.cfs.items()], [cf.initialPosition for _, cf in self.cfs.items()])

        # step as fast as possible
        max_dt = 0.0 if "max_dt" not in self._ros_parameters else self._ros_parameters["max_dt"]
        self.timer = self.create_timer(max_dt, self._timer_callback)

    def _timer_callback(self):
        states = [cf.getSetpoint() for _, cf in self.cfs.items()]
        self.backend.step(states)

    def _param_to_dict(self, param_ros):
        """
        Turn ROS2 parameters from the node into a dict
        """
        tree = {}
        for item in param_ros:
            t = tree
            for part in item.split('.'):
                if part == item.split('.')[-1]:
                    t = t.setdefault(part, param_ros[item].value)
                else:
                    t = t.setdefault(part, {})
        return tree

    def _emergency_callback(self, request, response, uri="all"):
        if uri == "all":
            for link_uri in self.uris:
                self.swarm._cfs[link_uri].cf.loc.send_emergency_stop()
        else:
            self.swarm._cfs[uri].cf.loc.send_emergency_stop()

        return response

    def _takeoff_callback(self, request, response, name="all"):
        """
        Service callback to take the crazyflie land to 
            a certain height in high level commander
        """

        duration = float(request.duration.sec) + \
            float(request.duration.nanosec / 1e9)
        self.get_logger().info(
            f"takeoff(height={request.height} m,"
            + f"duration={duration} s,"
            + f"group_mask={request.group_mask}) {name}"
        )
        cfs = self.cfs if name == "all" else {name: self.cfs[name]}
        for _, cf in cfs.items():
            cf.takeoff(request.height, duration, request.group_mask)

        return response

    def _land_callback(self, request, response, name="all"):
        """
        Service callback to make the crazyflie land to 
            a certain height in high level commander
        """
        duration = float(request.duration.sec) + \
            float(request.duration.nanosec / 1e9)
        self.get_logger().info(
            f"land(height={request.height} m,"
            + f"duration={duration} s,"
            + f"group_mask={request.group_mask})"
        )
        cfs = self.cfs if name == "all" else {name: self.cfs[name]}
        for _, cf in cfs.items():
            cf.land(request.height, duration, request.group_mask)

        return response

    def _go_to_callback(self, request, response, uri="all"):
        """
        Service callback to have the crazyflie go to 
            a certain position in high level commander
        """
        duration = float(request.duration.sec) + \
            float(request.duration.nanosec / 1e9)

        self.get_logger().info(
            "go_to(position=%f,%f,%f m, yaw=%f rad, duration=%f s, relative=%d, group_mask=%d)"
            % (
                request.goal.x,
                request.goal.y,
                request.goal.z,
                request.yaw,
                duration,
                request.relative,
                request.group_mask,
            )
        )
        if uri == "all":
            for link_uri in self.uris:
                self.swarm._cfs[link_uri].cf.high_level_commander.go_to(
                    request.goal.x,
                    request.goal.y,
                    request.goal.z,
                    request.yaw,
                    duration,
                    relative=request.relative,
                    group_mask=request.group_mask,
                )
        else:
            self.swarm._cfs[uri].cf.high_level_commander.go_to(
                request.goal.x,
                request.goal.y,
                request.goal.z,
                request.yaw,
                duration,
                relative=request.relative,
                group_mask=request.group_mask,
            )
        return response

    def _notify_setpoints_stop_callback(self, request, response, uri="all"):
        self.get_logger().info("Notify setpoint stop not yet implemented")
        return response

    def _upload_trajectory_callback(self, request, response, uri="all"):
        self.get_logger().info("Notify trajectory not yet implemented")
        return response
    
    def _start_trajectory_callback(self, request, response, uri="all"):
        self.get_logger().info("Start trajectory not yet implemented")
        return response

    def _cmd_vel_legacy_changed(self, msg, uri=""):
        """
        Topic update callback to control the attitude and thrust
            of the crazyflie with teleop
        """
        roll = msg.linear.y
        pitch = -msg.linear.x
        yawrate = msg.angular.z
        thrust = int(min(max(msg.linear.z, 0, 0), 60000))
        self.swarm._cfs[uri].cf.commander.send_setpoint(
            roll, pitch, yawrate, thrust)

    def _cmd_hover_changed(self, msg, uri=""):
        """
        Topic update callback to control the hover command
            of the crazyflie from the velocity multiplexer (vel_mux)
        """
        vx = msg.vx
        vy = msg.vy
        z = msg.z_distance
        yawrate = -1.0*degrees(msg.yaw_rate)
        self.swarm._cfs[uri].cf.commander.send_hover_setpoint(vx, vy, yawrate, z)
        self.get_logger().info(f"{uri}: Received hover topic {vx} {vy} {yawrate} {z}")


def main(args=None):

    rclpy.init(args=args)
    crazyflie_server = CrazyflieServer()

    rclpy.spin(crazyflie_server)

    crazyflie_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
