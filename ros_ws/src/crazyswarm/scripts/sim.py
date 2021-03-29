#!/usr/bin/env python3

import rospy
import tf2_ros
from std_msgs.msg import Empty
import geometry_msgs.msg
from visualization_msgs.msg import Marker

import yaml
import numpy as np
from crazyflie_driver.srv import *
from crazyflie_driver.msg import TrajectoryPolynomialPiece, FullState, Position, VelocityWorld
from pycrazyswarm.crazyflieSim import TimeHelper, Crazyflie


class TimeHelperROS:
    def __init__(self):
        self.start = rospy.Time.now()

    def time(self):
        return (rospy.Time.now() - self.start).to_sec()


class CrazyflieROS(Crazyflie):
    def __init__(self, id, initialPosition, timeHelper):
        super().__init__(id, initialPosition, timeHelper)

        prefix = "/cf" + str(id)
        rospy.Service(prefix + '/set_group_mask', SetGroupMask, self.handle_set_group_mask)
        rospy.Service(prefix + '/takeoff', Takeoff, self.handle_takeoff)
        rospy.Service(prefix + '/land', Land, self.handle_land)
        rospy.Service(prefix + '/go_to', GoTo, self.handle_go_to)
        rospy.Service(prefix + '/upload_trajectory', UploadTrajectory, self.handle_upload_trajectory)
        rospy.Service(prefix + '/start_trajectory', StartTrajectory, self.handle_start_trajectory)
        rospy.Service(prefix + '/notify_setpoints_stop', NotifySetpointsStop, self.handle_notify_setpoints_stop)
        rospy.Service(prefix + '/update_params', UpdateParams, self.handle_update_params)

        rospy.Subscriber(prefix + "/cmd_full_state", FullState, self.handle_cmd_full_state)
        rospy.Subscriber(prefix + "/cmd_position", Position, self.handle_cmd_position)
        rospy.Subscriber(prefix + "/cmd_stop", Empty, self.handle_cmd_stop)

        # LED support
        self.ledsPublisher = rospy.Publisher("/visualization_marker", Marker, queue_size=1)

        # hacky stop support
        self.stopped = False

    def handle_set_group_mask(self, req):
        self.setGroupMask(req.groupMask)
        return SetGroupMaskResponse()

    def handle_takeoff(self, req):
        print(req)
        self.takeoff(req.height, req.duration.to_sec(), req.groupMask)
        return TakeoffResponse()

    def handle_land(self, req):
        self.land(req.height, req.duration.to_sec(), req.groupMask)
        return LandResponse()

    def handle_go_to(self, req):
        goal = [req.goal.x, req.goal.y, req.goal.z]
        self.goTo(goal, req.yaw, req.duration.to_sec(), req.relative, req.groupMask)
        return GoToResponse()

    def handle_upload_trajectory(self, req):
        print("ERROR NOT IMPLEMENTED!")

    def handle_start_trajectory(self, req):
        print("ERROR NOT IMPLEMENTED!")

    def handle_notify_setpoints_stop(self, req):
        self.notifySetpointsStop(req.remainValidMillisecs)
        return NotifySetpointsStopResponse()

    def handle_update_params(self, req):
        print("Warning: Update params not implemented in simulation!", req)
        for param in req.params:
            if "ring/solid" in param:
                self.updateLED()
            if param == "ring/effect":
                v = rospy.get_param("/cf" + str(self.id) + "/ring/effect")
                if v == 0:
                    self.removeLED()

        return UpdateParamsResponse()

    def handle_cmd_full_state(self, msg):
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
        acc = [msg.acc.x, msg.acc.y, msg.acc.z]
        omega = [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
        # TODO: extract yaw from quat?
        self.cmdFullState(pos, vel, acc, 0, omega)

    def handle_cmd_position(self, msg):
        pos = [msg.x, msg.y, msg.z]
        self.cmdPosition(pos, msg.yaw)

    def handle_cmd_stop(self, msg):
        self.cmdStop()
        self.stopped = True
        self.removeLED()

    def removeLED(self):
        marker = Marker()
        marker.header.frame_id = "cf" + str(self.id)
        marker.ns = "LED"
        marker.id = self.id
        marker.action = marker.DELETE
        self.ledsPublisher.publish(marker)

    def updateLED(self):

        if rospy.has_param("/cf" + str(self.id) + "/ring/solidRed") and \
            rospy.has_param("/cf" + str(self.id) + "/ring/solidGreen") and \
            rospy.has_param("/cf" + str(self.id) + "/ring/solidBlue"):
            
            marker = Marker()
            marker.header.frame_id = "cf" + str(self.id)
            marker.ns = "LED"
            marker.id = self.id
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 0.2
            marker.color.r = rospy.get_param("/cf" + str(self.id) + "/ring/solidRed")
            marker.color.g = rospy.get_param("/cf" + str(self.id) + "/ring/solidGreen")
            marker.color.b = rospy.get_param("/cf" + str(self.id) + "/ring/solidBlue")
            marker.pose.orientation.w = 1.0
            marker.frame_locked = True
            self.ledsPublisher.publish(marker)

class CrazyflieServerROS:
    def __init__(self, timehelper, crazyflies_yaml="../launch/crazyflies.yaml"):
        """Initialize the server.

        Args:
            crazyflies_yaml (str): If ends in ".yaml", interpret as a path and load
                from file. Otherwise, interpret as YAML string and parse
                directly from string.
        """
        if crazyflies_yaml.endswith(".yaml"):
            with open(crazyflies_yaml, 'r') as ymlfile:
                cfg = yaml.safe_load(ymlfile)
        else:
            cfg = yaml.safe_load(crazyflies_yaml)

        self.crazyflies = []
        self.crazyfliesById = dict()
        for crazyflie in cfg["crazyflies"]:
            id = int(crazyflie["id"])
            initialPosition = crazyflie["initialPosition"]
            cf = CrazyflieROS(id, initialPosition, timeHelper)
            self.crazyflies.append(cf)
            self.crazyfliesById[id] = cf


if __name__ == "__main__":

    rospy.init_node("CrazyflieROSSim", anonymous=False)

    timeHelper = TimeHelperROS()
    srv = CrazyflieServerROS(timeHelper, rospy.get_param("crazyflies_yaml"))

    dt = 0.1
    rate = rospy.Rate(1/dt) # hz

    br = tf2_ros.TransformBroadcaster()
    transform = geometry_msgs.msg.TransformStamped()
    transform.header.frame_id = "world"
    transform.transform.rotation.x = 0
    transform.transform.rotation.y = 0
    transform.transform.rotation.z = 0
    transform.transform.rotation.w = 1

    while not rospy.is_shutdown():
        transform.header.stamp = rospy.Time.now()
        for cf in srv.crazyflies:
            cf.integrate(dt, 0, np.inf)
            if not cf.stopped:
                cfid = cf.id
                pos = cf.position()
                if np.isfinite(pos).all():
                    transform.child_frame_id = "/cf" + str(cfid)
                    transform.transform.translation.x = pos[0]
                    transform.transform.translation.y = pos[1]
                    transform.transform.translation.z = pos[2]
                    br.sendTransform(transform)
        for cf in srv.crazyflies:
            cf.flip()
        rate.sleep()