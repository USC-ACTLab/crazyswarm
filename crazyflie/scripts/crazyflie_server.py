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

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig

from crazyflie_interfaces.srv import Takeoff, Land, GoTo, RemoveLogging, AddLogging
from crazyflie_interfaces.srv import UploadTrajectory, StartTrajectory, NotifySetpointsStop
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, ParameterType
from crazyflie_interfaces.msg import Hover
from crazyflie_interfaces.msg import LogDataGeneric

from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import tf_transformations
from tf2_ros import TransformBroadcaster

from functools import partial
from math import degrees, radians, pi, cos, sin

cf_log_to_ros_param = {
    "uint8_t": ParameterType.PARAMETER_INTEGER,
    "uint16_t": ParameterType.PARAMETER_INTEGER,
    "uint32_t": ParameterType.PARAMETER_INTEGER,
    "int8_t": ParameterType.PARAMETER_INTEGER,
    "int16_t": ParameterType.PARAMETER_INTEGER,
    "int32_t": ParameterType.PARAMETER_INTEGER,
    "FP16": ParameterType.PARAMETER_DOUBLE,
    "float": ParameterType.PARAMETER_DOUBLE,
    "double": ParameterType.PARAMETER_DOUBLE,
}

class CrazyflieServer(Node):
    def __init__(self):
        super().__init__(
            "crazyflie_server",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # Turn ROS parameters into a dictionary
        self._ros_parameters = self._param_to_dict(self._parameters)

        self.uris = []
        self.cf_dict = {}
        self.uri_dict = {}
        self.type_dict = {}
        
        # Assign default topic types, variables and callbacks
        self.default_log_type = {"pose": PoseStamped,
                                "scan": LaserScan,
                                "odom": Odometry}
        self.default_log_vars = {"pose": ['stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z',
                                         'stabilizer.roll', 'stabilizer.pitch', 'stabilizer.yaw'],
                                "scan": ['range.front', 'range.left', 'range.back', 'range.right'],
                                "odom": ['stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z',
                                         'stabilizer.yaw', 'stabilizer.roll', 'stabilizer.pitch',
                                         'kalman.statePX', 'kalman.statePY', 'kalman.statePZ',
                                         'gyro.z', 'gyro.x', 'gyro.y']}
        self.default_log_fnc = {"pose": self._log_pose_data_callback,
                               "scan": self._log_scan_data_callback,
                               "odom": self._log_odom_data_callback}

        self.world_tf_name = "world"
        try:
            self.world_tf_name = self._ros_parameters["world_tf_name"]
        except KeyError:
            pass
        robot_data = self._ros_parameters["robots"]

        # Init a transform broadcaster
        self.tfbr = TransformBroadcaster(self)

        # Create easy lookup tables for uri, name and types
        for crazyflie in robot_data:
            if robot_data[crazyflie]["enabled"]:
                type_cf = robot_data[crazyflie]["type"]
                # do not include virtual objects
                connection = self._ros_parameters['robot_types'][type_cf].get("connection", "crazyflie")
                if connection == "crazyflie":
                    uri = robot_data[crazyflie]["uri"]
                    self.uris.append(uri)
                    self.cf_dict[uri] = crazyflie
                    self.uri_dict[crazyflie] = uri
                    self.type_dict[uri] = type_cf

        # Setup Swarm class cflib with connection callbacks and open the links
        factory = CachedCfFactory(rw_cache="./cache")
        self.swarm = Swarm(self.uris, factory=factory)
        self.swarm.fully_connected_crazyflie_cnt = 0

        # Initialize logging, services and parameters for each crazyflie
        for link_uri in self.uris:

            # Connect callbacks for different connection states of the crazyflie
            self.swarm._cfs[link_uri].cf.fully_connected.add_callback(
                self._fully_connected
            )
            self.swarm._cfs[link_uri].cf.disconnected.add_callback(
                self._disconnected)
            self.swarm._cfs[link_uri].cf.connection_failed.add_callback(
                self._connection_failed
            )

            self.swarm._cfs[link_uri].logging = {}

            cf_name = self.cf_dict[link_uri]
            cf_type = self.type_dict[link_uri]

            # check if logging is enabled at startup
            logging_enabled = False
            try:
                logging_enabled = self._ros_parameters['all']["firmware_logging"]["enabled"]
            except KeyError:
                pass
            try:
                logging_enabled = self._ros_parameters['robot_types'][cf_type]["firmware_logging"]["enabled"]
            except KeyError:
                pass
            try:
                logging_enabled = self._ros_parameters['robots'][cf_name]["firmware_logging"]["enabled"]
            except KeyError:
                pass

            self.swarm._cfs[link_uri].logging["enabled"] = logging_enabled

            # check if predefine log blocks can be logged and setup crazyflie logblocks and ROS2 publishers
            for default_log_name in self.default_log_type:
                prefix = default_log_name
                topic_type = self.default_log_type[default_log_name]
                list_logvar = self.default_log_vars[default_log_name]
                self._init_default_logblocks(prefix, link_uri, list_logvar, logging_enabled, topic_type)

            # Check for any custom_log topics
            custom_logging_enabled = False
            custom_log_topics = {}

            try:
                custom_log_topics = self._ros_parameters['all']["firmware_logging"]["custom_topics"]
                custom_logging_enabled = True
            except KeyError:
                pass
            try:
                custom_log_topics.update(
                    self._ros_parameters['robot_types'][cf_type]["firmware_logging"]["custom_topics"])
                custom_logging_enabled = True
            except KeyError:
                pass
            try:
                custom_log_topics.update(
                    self._ros_parameters['robots'][cf_name]["firmware_logging"]["custom_topics"])
                custom_logging_enabled = True
            except KeyError:
                pass

            self.swarm._cfs[link_uri].logging["custom_log_topics"] = {}
            self.swarm._cfs[link_uri].logging["custom_log_groups"] = {}
            self.swarm._cfs[link_uri].logging["custom_log_publisher"] = {}

            # Setup log blocks for each custom log and ROS2 publisher topics
            if custom_logging_enabled:
                for log_group_name in custom_log_topics:
                    frequency = custom_log_topics[log_group_name]["frequency"]
                    lg_custom = LogConfig(
                        name=log_group_name, period_in_ms=1000 / frequency)
                    for log_name in custom_log_topics[log_group_name]["vars"]:
                        lg_custom.add_variable(log_name)
                        # Don't know which type this needs to be in until we get the full toc
                    self.swarm._cfs[link_uri].logging["custom_log_publisher"][log_group_name] = "empty publisher"
                    self.swarm._cfs[link_uri].logging["custom_log_groups"][log_group_name] = {
                    }
                    self.swarm._cfs[link_uri].logging["custom_log_groups"][log_group_name]["log_config"] = lg_custom
                    self.swarm._cfs[link_uri].logging["custom_log_groups"][log_group_name]["vars"] = custom_log_topics[log_group_name]["vars"]
                    self.swarm._cfs[link_uri].logging["custom_log_groups"][log_group_name][
                        "frequency"] = custom_log_topics[log_group_name]["frequency"]

        # Now all crazyflies are initialized, open links!
        try:
            self.swarm.open_links()
        except Exception as e:
            # Close node if one of the Crazyflies can not be found
            self.get_logger().info("Error!: One or more Crazyflies can not be found. ")
            self.get_logger().info("Check if you got the right URIs, if they are turned on" +
                                   " or if your script have proper access to a Crazyradio PA")
            exit()
        
        # Create services for the entire swarm and each individual crazyflie
        self.create_service(Empty, "all/emergency", self._emergency_callback)
        self.create_service(Takeoff, "all/takeoff", self._takeoff_callback)
        self.create_service(Land, "all/land", self._land_callback)
        self.create_service(GoTo, "all/go_to", self._go_to_callback)
        self.create_service(StartTrajectory, "all/start_trajectory", self._start_trajectory_callback)

        for uri in self.cf_dict:
            name = self.cf_dict[uri]
            self.create_service(
                Empty, name +
                "/emergency", partial(self._emergency_callback, uri=uri)
            )
            self.create_service(
                Takeoff, name +
                "/takeoff", partial(self._takeoff_callback, uri=uri)
            )
            self.create_service(
                Land, name + "/land", partial(self._land_callback, uri=uri)
            )
            self.create_service(
                GoTo, name + "/go_to", partial(self._go_to_callback, uri=uri)
            )
            self.create_service(
                StartTrajectory, name + "/start_trajectory", partial(self._start_trajectory_callback, uri=uri)
            )
            self.create_service(
                UploadTrajectory, name + "/upload_trajectory", partial(self._upload_trajectory_callback, uri=uri) 
            )
            self.create_service(
                NotifySetpointsStop, name + "/notify_setpoints_stop", partial(self._notify_setpoints_stop_callback, uri=uri) 
            )
            self.create_subscription(
                Twist, name +
                "/cmd_vel_legacy", partial(self._cmd_vel_legacy_changed, uri=uri), 10
            )
            self.create_subscription(
                Hover, name +
                "/cmd_hover", partial(self._cmd_hover_changed, uri=uri), 10
            )

    def _init_default_logblocks(self, prefix, link_uri, list_logvar, global_logging_enabled, topic_type):
        """
        Prepare default logblocks as defined in crazyflies.yaml
        """
        cf_name = self.cf_dict[link_uri]
        cf_type = self.type_dict[link_uri]
        
        logging_enabled = False
        logging_freq = 10
        try:
            logging_freq = self._ros_parameters['all'][
                "firmware_logging"]["default_topics"][prefix]["frequency"]
            logging_enabled = True
        except KeyError:
            pass
        try:
            logging_freq = self._ros_parameters['robot_types'][cf_type][
                "firmware_logging"]["default_topics"][prefix]["frequency"]
            logging_enabled = True
        except KeyError:
            pass
        try:
            logging_freq = self._ros_parameters['robots'][cf_name][
                "firmware_logging"]["default_topics"][prefix]["frequency"]
            logging_enabled = True
        except KeyError:
            pass

        lg = LogConfig(
            name=prefix, period_in_ms=1000 / logging_freq)
        for logvar in list_logvar:
            if prefix == "odom":
                lg.add_variable(logvar, "FP16")
            else:
                lg.add_variable(logvar)

        self.swarm._cfs[link_uri].logging[prefix + "_logging_enabled"] = logging_enabled
        self.swarm._cfs[link_uri].logging[prefix + "_logging_freq"] = logging_freq
        self.swarm._cfs[link_uri].logging[prefix + "_log_config"] = lg
        if logging_enabled and global_logging_enabled:
            self.swarm._cfs[link_uri].logging[prefix + "_publisher"] = self.create_publisher(
                topic_type, self.cf_dict[link_uri] + "/" + prefix, 10)
        else:
            self.swarm._cfs[link_uri].logging[prefix + "_publisher"] = "empty"

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

    def _fully_connected(self, link_uri):
        """
        Called when all parameters have been updated
          and the full log toc has been received of the Crazyflie
        """
        self.get_logger().info(f" {link_uri} is fully connected!")

        self.swarm.fully_connected_crazyflie_cnt += 1

        if self.swarm.fully_connected_crazyflie_cnt == len(self.cf_dict):
            self.get_logger().info("All Crazyflies are fully connected!")
            self._init_parameters()
            self._init_logging()
            self.add_on_set_parameters_callback(self._parameters_callback)
        else:
            return

    def _disconnected(self, link_uri):
        self.get_logger().info(f" {link_uri} is disconnected!")

    def _connection_failed(self, link_uri, msg):
        self.get_logger().info(f"{link_uri} connection Failed")
        self.swarm.close_links()

    def _init_logging(self):
        """
        Sets up all the log blocks for the crazyflie and
           all the ROS2 publisher and parameters for logging
           at startup
        """
        for link_uri in self.uris:
            cf_handle = self.swarm._cfs[link_uri]
            cf = cf_handle.cf

            # Start logging for predefined logging
            for default_log_name in self.default_log_type:
                prefix = default_log_name
                if cf_handle.logging[prefix + "_logging_enabled"] and cf_handle.logging["enabled"]:
                    callback_fnc = self.default_log_fnc[prefix]
                    self._init_default_logging(prefix, link_uri, callback_fnc)
            
            # Start logging for costum logging blocks
            cf_handle.l_toc = cf.log.toc.toc
            if len(cf_handle.logging["custom_log_groups"]) != 0 and cf_handle.logging["enabled"]:

                for log_group_name, log_group_dict in cf_handle.logging["custom_log_groups"].items():
                    self.swarm._cfs[link_uri].logging["custom_log_publisher"][log_group_name] = self.create_publisher(
                        LogDataGeneric, self.cf_dict[link_uri] + "/" + log_group_name, 10)
                    lg_custom = log_group_dict['log_config']
                    try:
                        cf.log.add_config(lg_custom)
                        lg_custom.data_received_cb.add_callback(
                            partial(self._log_custom_data_callback, uri=link_uri))
                        lg_custom.error_cb.add_callback(
                            self._log_error_callback)
                        lg_custom.start()
                    except KeyError as e:
                        self.get_logger().info(f'{link_uri}: Could not start log configuration,'
                                               '{} not found in TOC'.format(str(e)))
                    except AttributeError:
                        self.get_logger().info(
                            f'{link_uri}: Could not add log config, bad configuration.')

                self.get_logger().info(f"{link_uri} setup custom logging")

            self.create_service(
                RemoveLogging, self.cf_dict[link_uri] + "/remove_logging", partial(self._remove_logging, uri=link_uri))
            self.create_service(
                AddLogging, self.cf_dict[link_uri] + "/add_logging", partial(self._add_logging, uri=link_uri))

        self.get_logger().info("All Crazyflies loggging are initialized")

    def _init_default_logging(self, prefix, link_uri, callback_fnc):
        """
        Sets up all the default log blocks and ROS2 publishers for the crazyflie
        """
        cf_handle = self.swarm._cfs[link_uri]
        cf = cf_handle.cf
        lg = cf_handle.logging[prefix + "_log_config"]
        try:
            cf.log.add_config(lg)
            lg.data_received_cb.add_callback(
                partial(callback_fnc, uri=link_uri))
            lg.error_cb.add_callback(self._log_error_callback)
            lg.start()
            frequency = cf_handle.logging[prefix + "_logging_freq"]
            self.declare_parameter(
                self.cf_dict[link_uri] + ".logs." + prefix + ".frequency.", frequency)
            self.get_logger().info(
                f"{link_uri} setup logging for {prefix} at freq {frequency}")
        except KeyError as e:
            self.get_logger().info(f'{link_uri}: Could not start log configuration,'
                                    '{} not found in TOC'.format(str(e)))
        except AttributeError:
            self.get_logger().info(
                f'{link_uri}: Could not add log config, bad configuration.')
    
    def _log_scan_data_callback(self, timestamp, data, logconf, uri):
        """
        Once multiranger range is retrieved from the Crazyflie, 
            send out the ROS2 topic for Scan
        """
        cf_name = self.cf_dict[uri]
        max_range = 3.49
        front_range = float(data.get('range.front'))/1000.0
        left_range = float(data.get('range.left'))/1000.0
        back_range = float(data.get('range.back'))/1000.0
        right_range = float(data.get('range.right'))/1000.0
        if front_range > max_range:
            front_range = float("inf")
        if left_range > max_range:
            left_range = float("inf")
        if right_range > max_range:
            right_range = float("inf")
        if back_range > max_range:
            back_range = float("inf")  
        self.ranges = [back_range, right_range, front_range, left_range]

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = cf_name
        msg.range_min = 0.01
        msg.range_max = 3.49
        msg.ranges = self.ranges
        msg.angle_min = -0.5 * 2* pi
        msg.angle_max =  0.25 * 2 * pi
        msg.angle_increment = 1.0 * pi/2
        self.swarm._cfs[uri].logging["scan_publisher"].publish(msg)

    def _log_pose_data_callback(self, timestamp, data, logconf, uri):
        """
        Once pose data is retrieved from the Crazyflie, 
            send out the ROS2 topic for Pose
        """

        cf_name = self.cf_dict[uri]

        x = data.get('stateEstimate.x')
        y = data.get('stateEstimate.y')
        z = data.get('stateEstimate.z')
        roll = radians(data.get('stabilizer.roll'))
        pitch = radians(-1.0 * data.get('stabilizer.pitch'))
        yaw = radians(data.get('stabilizer.yaw'))
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_tf_name
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.swarm._cfs[uri].logging["pose_publisher"].publish(msg)

        t_base = TransformStamped()
        t_base.header.stamp = self.get_clock().now().to_msg()
        t_base.header.frame_id = self.world_tf_name
        t_base.child_frame_id = cf_name
        t_base.transform.translation.x = x
        t_base.transform.translation.y = y
        t_base.transform.translation.z = z
        t_base.transform.rotation.x = q[0]
        t_base.transform.rotation.y = q[1]
        t_base.transform.rotation.z = q[2]
        t_base.transform.rotation.w = q[3]
        self.tfbr.sendTransform(t_base)

    def _log_odom_data_callback(self, timestamp, data, logconf, uri):
        """
        Once pose and velocity data is retrieved from the Crazyflie, 
            send out the ROS2 topic for Odometry in 2D (no z-axis)
        """
        cf_name = self.cf_dict[uri]

        x = data.get('stateEstimate.x')
        y = data.get('stateEstimate.y')
        z = data.get('stateEstimate.z')
        yaw = radians(data.get('stabilizer.yaw'))
        roll = radians(data.get('stabilizer.roll'))
        pitch = radians(data.get('stabilizer.pitch'))
        vx = data.get('kalman.statePX')
        vy = data.get('kalman.statePY')
        vz = data.get('kalman.statePY')
        yawrate = data.get('gyro.z')
        rollrate = data.get('gyro.x')
        pitchrate = data.get('gyro.y')

        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        msg = Odometry()
        msg.child_frame_id = cf_name
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_tf_name
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.linear.z = vz
        msg.twist.twist.angular.z = yawrate
        msg.twist.twist.angular.y = pitchrate
        msg.twist.twist.angular.x = rollrate

        self.swarm._cfs[uri].logging["odom_publisher"].publish(msg)

        t_base = TransformStamped()
        t_base.header.stamp = self.get_clock().now().to_msg()
        t_base.header.frame_id = 'odom'
        t_base.child_frame_id = cf_name
        t_base.transform.translation.x = x
        t_base.transform.translation.y = y
        t_base.transform.translation.z = z
        t_base.transform.rotation.x = q[0]
        t_base.transform.rotation.y = q[1]
        t_base.transform.rotation.z = q[2]
        t_base.transform.rotation.w = q[3]
        self.tfbr.sendTransform(t_base)

    def _log_custom_data_callback(self, timestamp, data, logconf, uri):
        """
        Once custom log block is retrieved from the Crazyflie, 
            send out the ROS2 topic for that same type of log
        """
        msg = LogDataGeneric()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.timestamp = timestamp
        for log_name in data:
            msg.values.append(data.get(log_name))

        self.swarm._cfs[uri].logging["custom_log_publisher"][logconf.name].publish(
            msg)

    def _log_error_callback(self, logconf, msg):
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _init_parameters(self):
        """
        Once custom log block is retrieved from the Crazyflie, 
            send out the ROS2 topic for that same type of log
        """
        for link_uri in self.uris:
            cf = self.swarm._cfs[link_uri].cf

            p_toc = cf.param.toc.toc

            for group in sorted(p_toc.keys()):
                for param in sorted(p_toc[group].keys()):
                    name = group + "." + param

                    # Check the parameter type 
                    elem = p_toc[group][param]
                    type_cf_param = elem.ctype
                    parameter_descriptor = ParameterDescriptor(type=cf_log_to_ros_param[type_cf_param])

                    # Check ros parameters if an parameter should be set
                    #   Parameter sets for individual robots has priority,
                    #   then robot types, then all (all robots)
                    set_param_value = None
                    try:
                        set_param_value = self._ros_parameters["all"]["firmware_params"][group][param]
                    except KeyError:
                        pass
                    try:
                        set_param_value = self._ros_parameters["robot_types"][self.cf_dict[link_uri]]["firmware_params"][group][param]
                    except KeyError:
                        pass
                    try:
                        set_param_value = self._ros_parameters["robots"][self.cf_dict[link_uri]]["firmware_params"][group][param]
                    except KeyError:
                        pass

                    if set_param_value is not None:
                        # If value is found in initial parameters,
                        # set crazyflie firmware value and declare value in ROS2 parameter
                        # Note: currently this is not possible to get the most recent from the
                        #       crazyflie with get_value due to threading.
                        cf.param.set_value(name, set_param_value)
                        self.get_logger().info(
                            f" {link_uri}: {name} is set to {set_param_value}"
                        )
                        self.declare_parameter(
                            self.cf_dict[link_uri] +
                            ".params." + group + "." + param,
                            value=set_param_value,
                            descriptor=parameter_descriptor,
                        )
                    else:
                        # If value is not found in initial parameter set
                        # get crazyflie paramter value and declare that value in ROS2 parameter

                        if cf_log_to_ros_param[type_cf_param] is ParameterType.PARAMETER_INTEGER:
                            cf_param_value = int(cf.param.get_value(name))
                        elif cf_log_to_ros_param[type_cf_param] is ParameterType.PARAMETER_DOUBLE:
                            cf_param_value = float(cf.param.get_value(name))

                        self.declare_parameter(
                            self.cf_dict[link_uri] +
                            ".params." + group + "." + param,
                            value=cf_param_value,
                            descriptor=parameter_descriptor,
                        )

        self.get_logger().info("All Crazyflies parameters are initialized")

    def _parameters_callback(self, params):
        """
        Sets up all the parameters for the crazyflie and
           translates it to ROS2 paraemeters at startup
        """
        for param in params:
            param_split = param.name.split(".")

            if param_split[0] in self.cf_dict.values():
                cf_name = param_split[0]
                if param_split[1] == "params":
                    name_param = param_split[2] + "." + param_split[3]
                    try:
                        self.swarm._cfs[self.uri_dict[cf_name]].cf.param.set_value(
                            name_param, param.value
                        )
                        self.get_logger().info(
                            f" {self.uri_dict[cf_name]}: {name_param} is set to {param.value}"
                        )
                        return SetParametersResult(successful=True)
                    except Exception as e:
                        self.get_logger().info(str(e))
                        return SetParametersResult(successful=False)
                if param_split[1] == "logs":
                    return SetParametersResult(successful=True)
        return SetParametersResult(successful=False)
    
    def _emergency_callback(self, request, response, uri="all"):
        if uri == "all":
            for link_uri in self.uris:
                self.swarm._cfs[link_uri].cf.loc.send_emergency_stop()
        else:
            self.swarm._cfs[uri].cf.loc.send_emergency_stop()

        return response

    def _takeoff_callback(self, request, response, uri="all"):
        """
        Service callback to take the crazyflie land to 
            a certain height in high level commander
        """

        duration = float(request.duration.sec) + \
            float(request.duration.nanosec / 1e9)
        self.get_logger().info(
            f"takeoff(height={request.height} m,"
            + f"duration={duration} s,"
            + f"group_mask={request.group_mask}) {uri}"
        )
        if uri == "all":
            for link_uri in self.uris:
                self.swarm._cfs[link_uri].cf.high_level_commander.takeoff(
                    request.height, duration
                )
        else:
            self.swarm._cfs[uri].cf.high_level_commander.takeoff(
                request.height, duration
            )

        return response

    def _land_callback(self, request, response, uri="all"):
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
        if uri == "all":
            for link_uri in self.uris:
                self.swarm._cfs[link_uri].cf.high_level_commander.land(
                    request.height, duration, group_mask=request.group_mask
                )
        else:
            self.swarm._cfs[uri].cf.high_level_commander.land(
                request.height, duration, group_mask=request.group_mask
            )

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

    def _remove_logging(self, request, response, uri="all"):
        """
        Service callback to remove logging blocks of the crazyflie
        """
        topic_name = request.topic_name
        if topic_name in self.default_log_type.keys():
            try:
                self.undeclare_parameter(
                    self.cf_dict[uri] + ".logs." + topic_name + ".frequency.")
                self.swarm._cfs[uri].logging[topic_name + "_log_config"].stop()
                self.destroy_publisher(
                    self.swarm._cfs[uri].logging[topic_name + "_publisher"])
                self.get_logger().info(f"{uri}: Remove {topic_name} logging")
            except rclpy.exceptions.ParameterNotDeclaredException:
                self.get_logger().info(
                    f"{uri}: No logblock of {topic_name} has been found ")
                response.success = False
                return response
        else:
            try:
                self.swarm._cfs[uri].logging["custom_log_groups"][topic_name]["log_config"].stop(
                )
                for log_name in self.swarm._cfs[uri].logging["custom_log_groups"][topic_name]["vars"]:
                    self.destroy_publisher(
                        self.swarm._cfs[uri].logging["custom_log_publisher"][topic_name])
                self.get_logger().info(f"{uri}: Remove {topic_name} logging")
            except rclpy.exceptions.ParameterNotDeclaredException:
                self.get_logger().info(
                    f"{uri}: No logblock of {topic_name} has been found ")
                response.success = False
                return response

        response.success = True
        return response

    def _add_logging(self, request, response, uri="all"):
        """
        Service callback to add logging blocks of the crazyflie
        """
        topic_name = request.topic_name
        frequency = request.frequency
        variables = request.vars
        if topic_name in self.default_log_type.keys():
            try:
                self.declare_parameter(
                    self.cf_dict[uri] + ".logs." + topic_name + ".frequency.", frequency)
                self.swarm._cfs[uri].logging[topic_name + "_publisher"] = self.create_publisher(
                    self.default_log_type[topic_name], self.cf_dict[uri] + "/" + topic_name, 10)
                self.swarm._cfs[uri].logging[topic_name + "_log_config"].period_in_ms = 1000 / frequency
                self.swarm._cfs[uri].logging[topic_name + "_log_config"].start()
                self.get_logger().info(f"{uri}: Add {topic_name} logging")
            except rclpy.exceptions.ParameterAlreadyDeclaredException:
                self.get_logger().info(
                    f"{uri}: The content the logging of {topic_name} has already started ")
                response.success = False
                return response
        else:
            try:
                self.declare_parameter(
                    self.cf_dict[uri] + ".logs." + topic_name + ".frequency.", frequency)
                self.declare_parameter(
                    self.cf_dict[uri] + ".logs." + topic_name + ".vars.", variables)
                lg_custom = LogConfig(
                    name=topic_name, period_in_ms=1000 / frequency)
                for log_name in variables:
                    lg_custom.add_variable(log_name)
                self.swarm._cfs[uri].logging["custom_log_publisher"][topic_name] = self.create_publisher(
                    LogDataGeneric, self.cf_dict[uri] + "/" + topic_name, 10)
                
                self.swarm._cfs[uri].cf.log.add_config(lg_custom)

                lg_custom.data_received_cb.add_callback(
                    partial(self._log_custom_data_callback, uri=uri))
                lg_custom.error_cb.add_callback(self._log_error_callback)
                lg_custom.start()

                self.swarm._cfs[uri].logging["custom_log_groups"][topic_name] = {}
                self.swarm._cfs[uri].logging["custom_log_groups"][topic_name]["log_config"] = lg_custom
                self.swarm._cfs[uri].logging["custom_log_groups"][topic_name]["vars"] = variables
                self.swarm._cfs[uri].logging["custom_log_groups"][topic_name]["frequency"] = frequency

                self.get_logger().info(f"{uri}: Add {topic_name} logging")
            except KeyError as e:
                self.get_logger().info(
                    f"{uri}: Failed to add {topic_name} logging")
                self.get_logger().info(str(e) + "is not in TOC")
                self.undeclare_parameter(self.cf_dict[uri] + ".logs." + topic_name + ".frequency.")
                self.undeclare_parameter(self.cf_dict[uri] + ".logs." + topic_name + ".vars.")
                response.success = False
                return response
            except rclpy.exceptions.ParameterAlreadyDeclaredException:
                self.get_logger().info(
                    f"{uri}: The content or part of the logging of {topic_name} has already started ")
                response.success = False
                return response

        response.success = True
        return response
        

def main(args=None):

    cflib.crtp.init_drivers()
    rclpy.init(args=args)
    crazyflie_server = CrazyflieServer()

    rclpy.spin(crazyflie_server)

    crazyflie_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
