import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import cflib.crtp  # noqa
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig

from crazyflie_interfaces.srv import Takeoff, Land, GoTo
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, ParameterType

from geometry_msgs.msg import Twist

import os
import yaml
from functools import partial


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
            self.swarm._cfs[link_uri].lg_pose = LogConfig(name='Pose', period_in_ms=100)
            self.swarm._cfs[link_uri].lg_pose.add_variable('stateEstimate.x')
            self.swarm._cfs[link_uri].lg_pose.add_variable('stateEstimate.y')
            self.swarm._cfs[link_uri].lg_pose.add_variable('stateEstimate.z')
            self.swarm._cfs[link_uri].lg_pose.add_variable('stateEstimateZ.quat')

        self.swarm.open_links()

        # Create services for the entire swarm and each individual crazyflie
        self.create_service(Takeoff, "/takeoff", self._takeoff_callback)
        self.create_service(Land, "/land", self._land_callback)
        self.create_service(GoTo, "/go_to", self._go_to_callback)

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
            cf = self.swarm._cfs[link_uri].cf
            lg_pose = self.swarm._cfs[link_uri].lg_pose
            try:
                cf.log.add_config(lg_pose)
                lg_pose.data_received_cb.add_callback(self._log_data_callback)
                lg_pose.error_cb.add_callback(self._log_error_callback)
                lg_pose.start()
                self.get_logger().info(f"{link_uri} setup Logging for Pose")
            except KeyError as e:
                self.get_logger().info(f'{link_uri}: Could not start log configuration,'
                        '{} not found in TOC'.format(str(e)))
            except AttributeError:
                self.get_logger().info(f'{link_uri}: Could not add log config, bad configuration.')

    def _log_data_callback(self, timestamp, data, logconf):
        print(f'[{timestamp}][{logconf.name}]: ', end='')
        for name, value in data.items():
            print(f'{name}: {value:3.3f} ', end='')
        print()

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
                    global_init_param_name = "all_robots.firmware_params." + name
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
        print(uri)
        roll = msg.linear.y
        pitch = -msg.linear.x
        yawrate = msg.angular.z
        thrust = int(min(max(msg.linear.z, 0, 0), 60000))
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
