from argparse import Namespace
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

from crazyflie_interfaces.srv import Takeoff, Land, GoTo
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, ParameterType
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist

import os
import yaml
from math import pi, isclose
from functools import partial
from threading import Event
import time


class CrazyflieServer(Node):
    def __init__(self):
        super().__init__("crazyflie_server", allow_undeclared_parameters=True,
                     automatically_declare_parameters_from_overrides=True)

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
        self.type_dict = {}
        self.param_dict = {}

        for crazyflie in data:
            uri = data[crazyflie]["uri"]
>>>>>>> reset parameters after fully connected
            self.uris.append(uri)
            self.cf_dict[uri] = crazyflie
            self.uri_dict[crazyflie] = uri
            type_cf = data[crazyflie]["type"]
            self.type_dict[uri] = type_cf


        # Setup Swarm class cflib with connection callbacks and open the links
        factory = CachedCfFactory(rw_cache="./cache")
        self.swarm = Swarm(self.uris, factory=factory)
        self.swarm.fully_connected_crazyflie_cnt = 0
        self.swarm.all_fully_connected = False
        for link_uri in self.uris:
            self.swarm._cfs[link_uri].cf.fully_connected.add_callback(self._fully_connected)
            self.swarm._cfs[link_uri].cf.disconnected.add_callback(self._disconnected)
            self.swarm._cfs[link_uri].cf.connection_failed.add_callback(
                self._connection_failed
            )
            self.swarm._cfs[link_uri].init_param_cnt = 0
            self.swarm._cfs[link_uri].total_param_cnt = 0

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
        
        self.param_set_event = Event()
        self.param_value_check = None

        self.param_list = {}
        for name, param in self._parameters.items():
            name_split = name.split('.')
            d_name, keys = name_split[0], name_split[1:]
            d = vars()[d_name]
            for key in keys:
                d = d.get(key, None)
                if d is None:
                    break
            print(d)

            print(len(name_split), name)
            if len(name_split) ==3:
                self.param_list.update({name_split[0]: {name_split[1]: {name_split[2]: param.value}}})
            elif len(name_split) ==4:
                self.param_list.update({name_split[0]: {name_split[1]: {name_split[2]: {name_split[3]: param.value}}}})
            elif len(name_split) ==5:
                self.param_list.update({name_split[0]: {name_split[1]: {name_split[2]: {name_split[3]: {name_split[4]: param.value}}}}})

    def _fully_connected(self, link_uri):
        self.get_logger().info(f" {link_uri} is fully connected!")

        self.swarm.fully_connected_crazyflie_cnt += 1
        self._set_cf_parameters_from_ROS(self.swarm._cfs[link_uri].cf,self.type_dict[link_uri], self.cf_dict[link_uri])

        if self.swarm.fully_connected_crazyflie_cnt == len(self.cf_dict):
            self.get_logger().info("All Crazyflies are is fully connected!")
            self.swarm.all_fully_connected = True
            #self._sync_parameters()
        else:
            return

    def _set_cf_parameters_from_ROS(self, cf, cf_type, cf_name):
        cf.param.add_update_callback(group=None, name=None, cb=self._param_callback)

        # First check global parameter list
        final_value = None
        print(self.param_list)
        for group in self.param_list['firmware_params'].items():
            print(group)
            for param, val in self.param_list['firmware_params'][group].items():
                print(param)
                name = group + '.' + param
                cf.param.set_value(name, val)
                print(name,val,'global')
        
        # Second check type parameter list
        if cf_type in self.param_list['crazyflie_types']:
            if 'firmware_params' in self.param_list['crazyflies'][cf_type]:
                dict_temp = self.param_list['crazyflie_types'][cf_type]['firmware_params']
                for group in dict_temp.keys():
                    for param, val in dict_temp[group].items():
                        name = group + '.' + param
                        cf.param.set_value(name, val)
                        print(name,val,'type')

        # Second check type parameter list
        if cf_name in  self.param_list['crazyflies']:
            if 'firmware_params' in self.param_list['crazyflies'][cf_name]:
                dict_temp = self.param_list['crazyflies'][cf_name]['firmware_params']
                for group in dict_temp.keys():
                    for param, val in dict_temp[group].items():
                        name = group + '.' + param
                        cf.param.set_value(name, val)
                        print(name,val,'individual')

    def _set_and_check_parameter(self, cf, name, value, max_tries):

        try_cnt = 0
        while True:
            try:
                self._compare_paramater_value(cf, name, value)
                break
            except Exception as e:
                print(str(e))
                time.sleep(1)
                try_cnt += 1
                if try_cnt>max_tries:
                    raise Exception('Too many tries for parameters')
                continue


    def _compare_paramater_value(self, cf, name, value):
        group = name.split('.')[0]
        param = name.split('.')[1]

        cf.param.set_value(name, value)
        cf.param.set_value(name, value)
        cf.param.set_value(name, value)

        time.sleep(1)

        get_value = cf.param.get_value(name)

        if get_value is None:
            raise Exception('value is none')

        if not isclose(float(get_value), float(value), abs_tol=1e-5):
            raise Exception(f'value {name} not the same {get_value}, {value}')


    def _get_parameter_value(self, cf, name ):
        cf_param_value = None
        while True:
            try: 
                cf_param_value = cf.param.get_value(name,timeout=1)
                break
            except Exception as e:
                print(str(e))
                time.sleep(1)
                continue 
        return cf_param_value

    def _param_callback(self, name, value):
        """Generic callback registered for all the groups"""
        print('{0}: {1}'.format(name, value))

    def _ros_param_callback(self, params):
        for param in params:
            print(vars(param))
        return SetParametersResult(successful=True)

    def _sync_parameters(self):
        
        for link_uri in self.uris:
            cf = self.swarm._cfs[link_uri].cf
            cf.param.add_update_callback(group=None, name=None, cb=self._param_callback)

            p_toc = cf.param.toc.toc

            for group in sorted(p_toc.keys()):
                for param in sorted(p_toc[group].keys()): 
                    name = group + '.' + param

                    final_value = None
 
                    # First check and set global parameters
                    global_parameter = self.get_parameter_or('firmware_params.'+name)
                    if global_parameter.value is not None:
                        final_value = global_parameter.value 
                    
                    #Then check and set Type parameters
                    type_parameter = self.get_parameter_or('crazyflie_types.' + self.type_dict[link_uri] + '.firmware_params.'+name)
                    if type_parameter.value is not None:
                        final_value = type_parameter.value

                    #Then check and set individual paramters
                    parameter = self.get_parameter_or('crazyflies.' + self.cf_dict[link_uri] + '.firmware_params.'+name)
                    if parameter.value is not None:
                        final_value = parameter.value

                    if final_value is not None:
                        cf.param.set_value(name, final_value)
                        time.sleep(3)
                        cf_param_value = cf.param.get_value(name,timeout=1)
                        print(name, cf_param_value, final_value)

                        #self._set_and_check_parameter(cf, name, final_value, 3)

                    elem = p_toc[group][param]
                    type_cf_param = elem.ctype
                    
                
                    cf_param_value = self._get_parameter_value(cf, name)


                    if cf_param_value is not None:
                        if type_cf_param == 'float':
                            value = float(cf_param_value)
                            parameter_descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
                        else:
                            value = int(cf_param_value)
                            parameter_descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER)
                    else:
                        raise Exception

                    self.declare_parameter(self.cf_dict[link_uri] + '.params.' + name, value=value,descriptor=parameter_descriptor)

        self.get_logger().info("All Crazyflies parameters are synced")

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

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(crazyflie_server)

    executor.spin()

    executor.shutdown()
    crazyflie_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
