from argparse import Namespace
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

from crazyswarm2_interfaces.srv import Takeoff, Land, GoTo

from geometry_msgs.msg import Twist

import os
import yaml
from math import pi
from functools import partial


class CrazyflieServer(Node):
    def __init__(self):
        super().__init__("crazyflie_server")

        # Read out crazyflie URIs 
        crazyflies_yaml = os.path.join(
            get_package_share_directory("crazyflie"), "config", "crazyflies.yaml"
        )
        with open(crazyflies_yaml) as f:
            data = yaml.safe_load(f)
        self.uris = []
<<<<<<< HEAD
        cf_dict = {}
        for crazyflie in data["robots"]:
            uri = data["robots"][crazyflie]["uri"]
=======
        self.cf_dict = {}
        self.uri_dict = {}
        for crazyflie in data:
            uri = data[crazyflie]["uri"]
>>>>>>> reset parameters after fully connected
            self.uris.append(uri)
            self.cf_dict[uri] = crazyflie
            self.uri_dict[crazyflie] = uri

        # Setup Swarm class cflib with connection callbacks and open the links
        factory = CachedCfFactory(rw_cache="./cache")
        self.swarm = Swarm(self.uris, factory=factory)
        self.swarm.all_fully_connected = False
        for link_uri in self.uris:
            self.swarm._cfs[link_uri].cf.connected.add_callback(self._connected)
            self.swarm._cfs[link_uri].cf.disconnected.add_callback(self._disconnected)
            self.swarm._cfs[link_uri].cf.connection_failed.add_callback(
                self._connection_failed
            )
            self.swarm._cfs[link_uri].init_param_cnt = 0
            self.swarm._cfs[link_uri].total_param_cnt = 0
            self.swarm._cfs[link_uri].fully_connected = False

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



    def _connected(self, link_uri):
        self.get_logger().info(f" {link_uri} is connected!")

        # Get the TOC of parameters
        # TODO: do this when new version cflib is out with fully_connected
        param_cnt = 0
        p_toc = self.swarm._cfs[link_uri].cf.param.toc.toc
        for group in sorted(p_toc.keys()):
            for param in sorted(p_toc[group].keys()):
                param_cnt += 1
                self.swarm._cfs[link_uri].cf.param.add_update_callback(group=group, name=param, cb=partial(self._param_callback, link_uri=link_uri ))
        self.get_logger().info(f" {link_uri} got param toc!")
        self.swarm._cfs[link_uri].total_param_cnt = param_cnt

    def _ros_parameters_callback(self, params):
        print(len(params))
        for param in params:
            #print(vars(param))
            split_string = param._name.split('/')
            cf_name = split_string[0]
            uri_link = self.uri_dict[cf_name]

            if (split_string[1]=='params'):
                param_name = split_string[2]+split_string[3]
                self.swarm._cfs[uri_link].cf.param.set_value(param_name, param.value)
        return SetParametersResult(successful=True)


    def _ros_parameter_to_cf_init(self):
        for link_uri in self.cf_dict:
            p_toc = self.swarm._cfs[link_uri].cf.param.toc.toc
            cf_name = self.cf_dict[link_uri]
            for group in sorted(p_toc.keys()):
                for param in sorted(p_toc[group].keys()):
                    ros_value = self.get_parameter(cf_name + '/params/' + group + '/' + param).value
                    try:
                        self.swarm._cfs[link_uri].cf.param.set_value(group + '.' + param, ros_value)
                    except Exception:
                        pass

    def _param_callback(self, name, value, link_uri='all'):
        if self.swarm.all_fully_connected is False:
            self.declare_parameter(self.cf_dict[link_uri]+'/params/'+name.replace(".","/"), value)
            try:
                self.declare_parameter('firmware_params/'+name.replace(".","/"), value)
            except Exception:
                pass

        self.swarm._cfs[link_uri].init_param_cnt += 1
        group = name.split()[0]
        self.swarm._cfs[link_uri].cf.param.remove_update_callback(group=group, name=name, cb=self._param_callback)
        if self.swarm._cfs[link_uri].init_param_cnt == self.swarm._cfs[link_uri].total_param_cnt:
            self.get_logger().info(f" {link_uri} is fully connected!")
            self.swarm._cfs[link_uri].fully_connected = True

        all_connected = False
        for uri in self.swarm._cfs:
            if self.swarm._cfs[uri].fully_connected is True:
                all_connected = True
            else:
                all_connected = False
                break

            
        if all_connected:
            self.get_logger().info("all crazyflies are fully connected!")
            self.add_on_set_parameters_callback(self._ros_parameters_callback)
            self._ros_parameter_to_cf_init()
            self.swarm.all_fully_connected = True


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
