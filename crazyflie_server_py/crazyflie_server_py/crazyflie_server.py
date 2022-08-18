import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import cflib.crtp  
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig

from crazyflie_interfaces.srv import Takeoff, Land, GoTo, RemoveLogging
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, ParameterType

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, UInt8, UInt16, UInt32, Int8, Int16, Int32, Float32

import tf_transformations

import os
import yaml
from functools import partial
from math import radians

cf_log_to_ros_topic = {
    "uint8_t": UInt8,
    "uint16_t": UInt16,
    "uint32_t": UInt32,
    "int8_t ": Int8,
    "int16_t": Int16,
    "int32_t": Int32,
    "float": Float32,
}

class CrazyflieServer(Node):
    def __init__(self):
        super().__init__(
            "crazyflie_server",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # Read out crazyflie URIs
        crazyflies_yaml = os.path.join(
            get_package_share_directory("crazyflie"), "config", "crazyflies.yaml"
        )
        with open(crazyflies_yaml) as f:
            data = yaml.safe_load(f)

        self.uris = []
        self.cf_dict = {}
        self.uri_dict = {}
        self.type_dict = {}

        for crazyflie in data["robots"]:
            uri = data["robots"][crazyflie]["uri"]
            self.uris.append(uri)
            self.cf_dict[uri] = crazyflie
            self.uri_dict[crazyflie] = uri
            type_cf = data["robots"][crazyflie]["type"]
            self.type_dict[uri] = type_cf

        # Setup Swarm class cflib with connection callbacks and open the links
        factory = CachedCfFactory(rw_cache="./cache")
        self.swarm = Swarm(self.uris, factory=factory)
        self.swarm.fully_connected_crazyflie_cnt = 0
        self.swarm.all_fully_connected = False
        self.swarm.robot_types = {}

        self.swarm.custom_log_topics = {}
        self._pose_logging_enabled = False
        self._pose_logging_freq = 10

        self._ros_parameters = self._param_to_dict(self._parameters)
        
        for link_uri in self.uris:
            self.swarm._cfs[link_uri].cf.fully_connected.add_callback(
                self._fully_connected
            )
            self.swarm._cfs[link_uri].cf.disconnected.add_callback(self._disconnected)
            self.swarm._cfs[link_uri].cf.connection_failed.add_callback(
                self._connection_failed
            )
            self.swarm._cfs[link_uri].init_param_cnt = 0
            self.swarm._cfs[link_uri].total_param_cnt = 0
            self.swarm._cfs[link_uri].logging = {}

            cf_name = self.cf_dict[link_uri]
            cf_type = self.type_dict[link_uri]

            #check if logging is enabled
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

            #check if pose can be logged
            pose_logging_enabled = False
            pose_logging_freq = 10
            try: 
                pose_logging_freq = self._ros_parameters['all']["firmware_logging"]["default_topics"]["pose"]["frequency"]
                pose_logging_enabled = True
            except KeyError:
                pass
            try: 
                pose_logging_freq = self._ros_parameters['robot_types'][cf_type]["firmware_logging"]["default_topics"]["pose"]["frequency"]
                pose_logging_enabled = True
            except KeyError:
                pass
            try: 
                pose_logging_freq = self._ros_parameters['robots'][cf_name]["firmware_logging"]["default_topics"]["pose"]["frequency"]
                pose_logging_enabled = True
            except KeyError:
                pass

            lg_pose = LogConfig(name='Pose', period_in_ms=1000 / pose_logging_freq)            
            lg_pose.add_variable('stateEstimate.x')
            lg_pose.add_variable('stateEstimate.y')
            lg_pose.add_variable('stateEstimate.z')
            lg_pose.add_variable('stabilizer.roll', 'float')
            lg_pose.add_variable('stabilizer.pitch', 'float')
            lg_pose.add_variable('stabilizer.yaw', 'float')
            self.swarm._cfs[link_uri].logging["pose_logging_enabled"] = pose_logging_enabled
            self.swarm._cfs[link_uri].logging["pose_logging_freq"] = pose_logging_freq
            self.swarm._cfs[link_uri].logging["pose_log_config"] = lg_pose
            self.swarm._cfs[link_uri].logging["pose_publisher"] = self.create_publisher(PoseStamped, self.cf_dict[link_uri] + "/pose", 10)

            # Check for any custom_log_topics names
            #check if pose can be logged
            custom_logging_enabled = False
            custom_log_topics = {}
            
            try: 
                custom_log_topics = self._ros_parameters['all']["firmware_logging"]["custom_topics"]
                custom_logging_enabled = True
            except KeyError:
                pass
            try: 
                custom_log_topics.update(self._ros_parameters['robot_types'][cf_type]["firmware_logging"]["custom_topics"])
                custom_logging_enabled = True
            except KeyError:
                pass
            try: 
                custom_log_topics.update(self._ros_parameters['robots'][cf_name]["firmware_logging"]["custom_topics"])
                custom_logging_enabled = True
            except KeyError:
                pass

            self.swarm._cfs[link_uri].logging["custom_log_topics"] = {}
            self.swarm._cfs[link_uri].logging["custom_log_groups"] = {}
            self.swarm._cfs[link_uri].logging["lg_custom"] = []
            self.swarm._cfs[link_uri].logging["custom_log_publisher"] = {}

            if custom_logging_enabled:
                for log_group_name in custom_log_topics:
                    frequency = custom_log_topics[log_group_name]["frequency"]
                    lg_custom = LogConfig(name=log_group_name, period_in_ms=1000 / frequency)
                    for log_name in custom_log_topics[log_group_name]["vars"]:
                        lg_custom.add_variable(log_name)
                        # Don't know which type this needs to be in until we get the full toc
                        self.swarm._cfs[link_uri].logging["custom_log_publisher"][log_name] = "empty publisher"
                    self.swarm._cfs[link_uri].logging["custom_log_groups"][log_group_name] = {}
                    self.swarm._cfs[link_uri].logging["custom_log_groups"][log_group_name]["log_config"] = lg_custom
                    self.swarm._cfs[link_uri].logging["custom_log_groups"][log_group_name]["vars"] = custom_log_topics[log_group_name]["vars"]

        self.swarm.open_links()

        # Create services for the entire swarm and each individual crazyflie
        self.create_service(Takeoff, "all/takeoff", self._takeoff_callback)
        self.create_service(Land, "all/land", self._land_callback)
        self.create_service(GoTo, "all/go_to", self._go_to_callback)

        for uri in self.cf_dict:
            name = self.cf_dict[uri]
            self.create_service(
                Takeoff, name + "/takeoff", partial(self._takeoff_callback, uri=uri)
            )
            self.create_service(
                Land, name + "/land", partial(self._land_callback, uri=uri)
            )
            self.create_service(
                GoTo, name + "/go_to", partial(self._go_to_callback, uri=uri)
            )
            self.create_subscription(
                Twist, name + "/cmd_vel", partial(self._cmd_vel_changed, uri=uri), 10
            )

    def _param_to_dict(self, param_ros):
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
        self.get_logger().info(f" {link_uri} is fully connected!")

        self.swarm.fully_connected_crazyflie_cnt += 1

        if self.swarm.fully_connected_crazyflie_cnt == len(self.cf_dict):
            self.get_logger().info("All Crazyflies are fully connected!")
            self.swarm.all_fully_connected = True
            self._sync_parameters()
            self._sync_logging()
            self.add_on_set_parameters_callback(self.parameters_callback)
        else:
            return

    def _sync_logging(self):
        for link_uri in self.uris:
            cf_handle = self.swarm._cfs[link_uri]
            cf = cf_handle.cf

            if cf_handle.logging["pose_logging_enabled"] and cf_handle.logging["enabled"]:
                lg_pose = cf_handle.logging["pose_log_config"]
                try:
                    cf.log.add_config(lg_pose)
                    lg_pose.data_received_cb.add_callback(partial(self._log_pose_data_callback, uri=link_uri)) 
                    lg_pose.error_cb.add_callback(self._log_error_callback)
                    lg_pose.start()
                    frequency = cf_handle.logging["pose_logging_freq"]
                    self.get_logger().info(f"{link_uri} setup logging for pose at freq {frequency}")
                except KeyError as e:
                    self.get_logger().info(f'{link_uri}: Could not start log configuration,'
                            '{} not found in TOC'.format(str(e)))
                except AttributeError:
                    self.get_logger().info(f'{link_uri}: Could not add log config, bad configuration.')

            cf_handle.l_toc = cf.log.toc.toc
            if len(cf_handle.logging["custom_log_groups"]) != 0 and cf_handle.logging["enabled"]:

                for log_name in cf_handle.logging["custom_log_publisher"]:

                    log_type = cf.log.toc.toc[log_name.split('.')[0]][log_name.split('.')[1]].ctype
                    self.swarm._cfs[link_uri].logging["custom_log_publisher"][log_name] =  self.create_publisher(cf_log_to_ros_topic[log_type], self.cf_dict[link_uri] + "/" + log_name.split('.')[0] + "/" + log_name.split('.')[1], 10)

                for log_group_dict in cf_handle.logging["custom_log_groups"].values():
                    lg_custom = log_group_dict['log_config']
                    try:
                        cf.log.add_config(lg_custom)
                        lg_custom.data_received_cb.add_callback(partial(self._log_custom_data_callback, uri=link_uri)) 
                        lg_custom.error_cb.add_callback(self._log_error_callback)
                        lg_custom.start()
                    except KeyError as e:
                        self.get_logger().info(f'{link_uri}: Could not start log configuration,'
                                '{} not found in TOC'.format(str(e)))
                    except AttributeError:
                        self.get_logger().info(f'{link_uri}: Could not add log config, bad configuration.')
                self.get_logger().info(f"{link_uri} setup custom logging")

                self.create_service(RemoveLogging, self.cf_dict[link_uri] + "/remove_logging", partial(self._remove_logging, uri=link_uri))


    def _log_pose_data_callback(self, timestamp, data, logconf, uri):

        x = data.get('stateEstimate.x')
        y = data.get('stateEstimate.y')
        z = data.get('stateEstimate.z')
        roll = radians(data.get('stabilizer.roll'))
        pitch = radians(-1.0 * data.get('stabilizer.pitch'))
        yaw = radians(data.get('stabilizer.yaw'))

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.swarm._cfs[uri].logging["pose_publisher"].publish(msg)

    def _log_custom_data_callback(self, timestamp, data, logconf, uri):

        for log_name in data:
            log_type = self.swarm._cfs[uri].l_toc[log_name.split('.')[0]][log_name.split('.')[1]].ctype
            msg = cf_log_to_ros_topic[log_type]()
            msg.data = data.get(log_name)
            self.swarm._cfs[uri].logging["custom_log_publisher"][log_name].publish(msg)            

    def _log_error_callback(self, logconf, msg):
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _sync_parameters(self):

        for link_uri in self.uris:
            cf = self.swarm._cfs[link_uri].cf

            p_toc = cf.param.toc.toc

            for group in sorted(p_toc.keys()):
                for param in sorted(p_toc[group].keys()):
                    name = group + "." + param

                    final_value = None

                    # First check and set global parameters
                    global_init_param_name = "all.firmware_params." + name
                    global_parameter = self.get_parameter_or(global_init_param_name)
                    if global_parameter.value is not None:
                        final_value = global_parameter.value

                    # Then check and set Type parameters
                    type_init_param_name = (
                        "robot_types."
                        + self.type_dict[link_uri]
                        + ".firmware_params."
                        + name
                    )
                    type_parameter = self.get_parameter_or(type_init_param_name)
                    if type_parameter.value is not None:
                        final_value = type_parameter.value

                    # Then check and set individual paramters
                    cf_init_param_name = (
                        "robots."
                        + self.cf_dict[link_uri]
                        + ".firmware_params."
                        + name
                    )
                    cf_parameter = self.get_parameter_or(cf_init_param_name)
                    if cf_parameter.value is not None:
                        final_value = cf_parameter.value

                    elem = p_toc[group][param]
                    type_cf_param = elem.ctype
                    if type_cf_param == "float":
                        parameter_descriptor = ParameterDescriptor(
                            type=ParameterType.PARAMETER_DOUBLE
                        )
                    else:
                        parameter_descriptor = ParameterDescriptor(
                            type=ParameterType.PARAMETER_INTEGER
                        )

                    if final_value is not None:
                        # If value is found in initial parameters,
                        # set crazyflie firmware value and declare value in ROS2 parameter
                        # Note: currently this is not possible to get the most recent from the
                        #       crazyflie with get_value due to threading.
                        cf.param.set_value(name, final_value)
                        self.get_logger().info(
                            f" {link_uri}: {name} is set to {final_value}"
                        )
                        self.declare_parameter(
                            self.cf_dict[link_uri] + "/params/" + group + "/" + param,
                            value=final_value,
                            descriptor=parameter_descriptor,
                        )
                    else:
                        # If value is net found in initial parameter set
                        # get crazyflie paramter value and declare that value in ROS2 parameter

                        cf_param_value = cf.param.get_value(name)

                        self.declare_parameter(
                            self.cf_dict[link_uri] + "/params/" + group + "/" + param,
                            value=cf_param_value,
                            descriptor=parameter_descriptor,
                        )

        self.get_logger().info("All Crazyflies parameters are synced")

    def parameters_callback(self, params):
        for param in params:
            param_split = param.name.split("/")

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

                    except Exception as e:
                        self.get_logger().info(str(e))   

                        return SetParametersResult(successful=False)
        return SetParametersResult(successful=False)

    def _disconnected(self, link_uri):
        self.get_logger().info(f" {link_uri} is disconnected!")

    def _connection_failed(self, link_uri, msg):
        self.get_logger().info(f"{link_uri} connection Failed")
        self.swarm.close_links()

    def _takeoff_callback(self, request, response, uri="all"):
        duration = float(request.duration.sec) + float(request.duration.nanosec / 1e9)
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
        duration = float(request.duration.sec) + float(request.duration.nanosec / 1e9)
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
        duration = float(request.duration.sec) + float(request.duration.nanosec / 1e9)
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

    def _cmd_vel_changed(self, msg, uri=""):
        roll = msg.linear.y
        pitch = -msg.linear.x
        yawrate = msg.angular.z
        thrust = int(min(max(msg.linear.z, 0, 0), 60000))
        self.swarm._cfs[uri].cf.commander.send_setpoint(roll, pitch, yawrate, thrust)

    def _remove_logging(self, request, response, uri="all"):
        topic_name = request.topic_name
        if topic_name == "pose":
            self.swarm._cfs[uri].logging["pose_log_config"].stop()
            self.destroy_publisher(self.swarm._cfs[uri].logging["pose_publisher"])
            self.get_logger().info("Remove pose logging")
        else:
            self.swarm._cfs[uri].logging["custom_log_groups"][topic_name]["log_config"].stop()
            for log_name in self.swarm._cfs[uri].logging["custom_log_groups"][topic_name]["vars"]:
                self.destroy_publisher(self.swarm._cfs[uri].logging["custom_log_publisher"][log_name])
            self.get_logger().info(f"Remove {topic_name} logging")

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
