#!/usr/bin/env python3

"""
A crazyflie server for simulation.


    2022 - Wolfgang Hönig (TU Berlin)
"""

import rclpy
from rclpy.node import Node
import rowan
import importlib

from crazyflie_interfaces.srv import Takeoff, Land, GoTo
from crazyflie_interfaces.srv import UploadTrajectory, StartTrajectory, NotifySetpointsStop
from crazyflie_interfaces.msg import Hover, FullState

from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

from functools import partial

# import BackendRviz from .backend_rviz
# from .backend import *
# from .backend.none import BackendNone
from .crazyflie_sil import CrazyflieSIL, TrajectoryPolynomialPiece
from .sim_data_types import State, Action


class CrazyflieServer(Node):
    def __init__(self):
        super().__init__(
            "crazyflie_server",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # Turn ROS parameters into a dictionary
        self._ros_parameters = self._param_to_dict(self._parameters)
        self.cfs = dict()
        
        self.world_tf_name = "world"
        try:
            self.world_tf_name = self._ros_parameters["world_tf_name"]
        except KeyError:
            pass
        robot_data = self._ros_parameters["robots"]

        # Parse robots
        names = []
        initial_states = []
        for cfname in robot_data:
            if robot_data[cfname]["enabled"]:
                type_cf = robot_data[cfname]["type"]
                # do not include virtual objects
                connection = self._ros_parameters['robot_types'][type_cf].get("connection", "crazyflie")
                if connection == "crazyflie":
                    names.append(cfname)
                    pos = robot_data[cfname]["initial_position"]
                    initial_states.append(State(pos))

        # initialize backend by dynamically loading the module
        backend_name = self._ros_parameters["sim"]["backend"]
        module = importlib.import_module(".backend." + backend_name, package="crazyflie_sim")
        class_ = getattr(module, "Backend")
        self.backend = class_(self, names, initial_states)

        # initialize visualizations by dynamically loading the modules
        self.visualizations = []
        for vis_key in self._ros_parameters["sim"]["visualizations"]:
            if self._ros_parameters["sim"]["visualizations"][vis_key]["enabled"]:
                module = importlib.import_module(".visualization." + str(vis_key), package="crazyflie_sim")
                class_ = getattr(module, "Visualization")
                vis = class_(self, self._ros_parameters["sim"]["visualizations"][vis_key], names, initial_states)
                self.visualizations.append(vis)

        controller_name = backend_name = self._ros_parameters["sim"]["controller"]

        # create robot SIL objects
        for name, initial_state in zip(names, initial_states):
            self.cfs[name] = CrazyflieSIL(
                name,
                initial_state.pos,
                controller_name,
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
            self.create_subscription(
                FullState, name +
                "/cmd_full_state", partial(self._cmd_full_state_changed, name=name), 10
            )

        # step as fast as possible
        max_dt = 0.0 if "max_dt" not in self._ros_parameters["sim"] else self._ros_parameters["sim"]["max_dt"]
        self.timer = self.create_timer(max_dt, self._timer_callback)
        self.is_shutdown = False

    def on_shutdown_callback(self):
        if not self.is_shutdown:
            self.backend.shutdown()
            for visualization in self.visualizations:
                visualization.shutdown()

            self.is_shutdown = True

    def _timer_callback(self):
        # update setpoint
        states_desired = [cf.getSetpoint() for _, cf in self.cfs.items()]

        # execute the control loop
        actions = [cf.executeController()  for _, cf in self.cfs.items()]

        # execute the physics simulator
        states_next = self.backend.step(states_desired, actions)

        # update the resulting state
        for state, (_, cf) in zip(states_next, self.cfs.items()):
            cf.setState(state)

        for vis in self.visualizations:
            vis.step(self.backend.time(), states_next, states_desired, actions)

    def _param_to_dict(self, param_ros):
        """
        Turn ROS 2 parameters from the node into a dict
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

    def _emergency_callback(self, request, response, name="all"):
        self.get_logger().info("emergency not yet implemented")

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

    def _go_to_callback(self, request, response, name="all"):
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
        cfs = self.cfs if name == "all" else {name: self.cfs[name]}
        for _, cf in cfs.items():
            cf.goTo([request.goal.x, request.goal.y, request.goal.z], 
                    request.yaw, duration, request.relative, request.group_mask)
        
        return response

    def _notify_setpoints_stop_callback(self, request, response, name="all"):
        self.get_logger().info("Notify setpoint stop not yet implemented")
        return response

    def _upload_trajectory_callback(self, request, response, name="all"):
        self.get_logger().info("Upload trajectory(id=%d)" % (request.trajectory_id))

        cfs = self.cfs if name == "all" else {name: self.cfs[name]}
        for _, cf in cfs.items():
            pieces = []
            for piece in request.pieces:
                poly_x = piece.poly_x
                poly_y = piece.poly_y
                poly_z = piece.poly_z
                poly_yaw = piece.poly_yaw
                duration = float(piece.duration.sec) + \
                    float(piece.duration.nanosec / 1e9)
                pieces.append(TrajectoryPolynomialPiece(poly_x, poly_y, poly_z, poly_yaw, duration))
            cf.uploadTrajectory(request.trajectory_id, request.piece_offset, pieces)

        return response
    
    def _start_trajectory_callback(self, request, response, name="all"):
        self.get_logger().info(
            "start_trajectory(id=%d, timescale=%f, reverse=%d, relative=%d, group_mask=%d)"
            % (
                request.trajectory_id,
                request.timescale,
                request.reversed,
                request.relative,
                request.group_mask,
            )
        )
        cfs = self.cfs if name == "all" else {name: self.cfs[name]}
        for _, cf in cfs.items():
            cf.startTrajectory(request.trajectory_id, request.timescale, request.reversed, request.relative, request.group_mask)

        return response

    def _cmd_vel_legacy_changed(self, msg, name=""):
        """
        Topic update callback to control the attitude and thrust
            of the crazyflie with teleop
        """
        self.get_logger().info("cmd_vel_legacy not yet implemented")


    def _cmd_hover_changed(self, msg, name=""):
        """
        Topic update callback to control the hover command
            of the crazyflie from the velocity multiplexer (vel_mux)
        """
        self.get_logger().info("cmd_hover not yet implemented")

    def _cmd_full_state_changed(self, msg, name):
        q = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]
        rpy = rowan.to_euler(q)

        self.cfs[name].cmdFullState(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
            [msg.acc.x, msg.acc.y, msg.acc.z],
            rpy[2],
            [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])


def main(args=None):

    rclpy.init(args=args)
    crazyflie_server = CrazyflieServer()
    rclpy.get_default_context().on_shutdown(crazyflie_server.on_shutdown_callback)

    try:
        rclpy.spin(crazyflie_server)
    except KeyboardInterrupt:
        crazyflie_server.on_shutdown_callback()
    finally:
        rclpy.try_shutdown()
        crazyflie_server.destroy_node()


if __name__ == "__main__":
    main()
