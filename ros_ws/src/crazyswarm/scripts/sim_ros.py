#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
import geometry_msgs.msg
from visualization_msgs.msg import Marker

import numpy as np
from crazyswarm.srv import *
from crazyswarm.msg import TrajectoryPolynomialPiece, FullState, Position, VelocityWorld
from pycrazyswarm.crazyflieSim import Crazyflie, CrazyflieServer
import uav_trajectory


class TimeHelperROS:
    def __init__(self):
        self.start = rospy.Time.now()

    def time(self):
        return (rospy.Time.now() - self.start).to_sec()


class CrazyflieROSSim:
    def __init__(self, id, initialPosition, timeHelper):
        self.cfsim = Crazyflie(id, initialPosition, timeHelper)

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
        rospy.Subscriber(prefix + "/cmd_stop", EmptyMsg, self.handle_cmd_stop)

        # LED support
        self.ledsPublisher = rospy.Publisher("/visualization_marker", Marker, queue_size=1)

        # hacky stop support
        self.stopped = False

    def handle_set_group_mask(self, req):
        self.cfsim.setGroupMask(req.groupMask)
        return SetGroupMaskResponse()

    def handle_takeoff(self, req):
        self.cfsim.takeoff(req.height, req.duration.to_sec(), req.groupMask)
        return TakeoffResponse()

    def handle_land(self, req):
        self.cfsim.land(req.height, req.duration.to_sec(), req.groupMask)
        return LandResponse()

    def handle_go_to(self, req):
        goal = [req.goal.x, req.goal.y, req.goal.z]
        self.cfsim.goTo(goal, req.yaw, req.duration.to_sec(), req.relative, req.groupMask)
        return GoToResponse()

    def handle_upload_trajectory(self, req):
        trajectory = uav_trajectory.Trajectory()
        trajectory.duration = 0
        trajectory.polynomials = []
        for piece in req.pieces:
            poly = uav_trajectory.Polynomial4D(
                piece.duration.to_sec(), piece.poly_x, piece.poly_y, piece.poly_z, piece.poly_yaw)
            trajectory.polynomials.append(poly)
            trajectory.duration += poly.duration
        self.cfsim.uploadTrajectory(req.trajectoryId, req.pieceOffset, trajectory)
        return UploadTrajectoryResponse()

    def handle_start_trajectory(self, req):
        self.cfsim.startTrajectory(req.trajectoryId, req.timescale, req.reversed, req.relative, req.groupMask)
        return StartTrajectoryResponse()

    def handle_notify_setpoints_stop(self, req):
        self.cfsim.notifySetpointsStop(req.remainValidMillisecs)
        return NotifySetpointsStopResponse()

    def handle_update_params(self, req):
        for param in req.params:
            if "ring/solid" in param:
                self.updateLED()
            if param == "ring/effect":
                v = rospy.get_param("/cf" + str(self.id) + "/ring/effect")
                if v == 0:
                    self.removeLED()
            else:
                rospy.logwarn("Updating param {} not implemented in simulation!".format(param))

        return UpdateParamsResponse()

    def handle_cmd_full_state(self, msg):
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
        acc = [msg.acc.x, msg.acc.y, msg.acc.z]
        omega = [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
        xyzw = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(xyzw)
        self.cfsim.cmdFullState(pos, vel, acc, yaw, omega)

    def handle_cmd_position(self, msg):
        pos = [msg.x, msg.y, msg.z]
        self.cfsim.cmdPosition(pos, msg.yaw)

    def handle_cmd_stop(self, msg):
        self.cfsim.cmdStop()
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


class CrazyflieServerROSSim:
    def __init__(self, timehelper):
        """Initialize the server.
        """
        # Populate crazyflies
        self.crazyflies = []
        for crazyflie in rospy.get_param("crazyflies"):
            id = int(crazyflie["id"])
            initialPosition = crazyflie["initialPosition"]
            cf = CrazyflieROSSim(id, initialPosition, timehelper)
            self.crazyflies.append(cf)

        rospy.Service('/emergency', EmptySrv, self.handle_emergency)
        rospy.Service('/takeoff', Takeoff, self.handle_takeoff)
        rospy.Service('/land', Land, self.handle_land)
        rospy.Service('/go_to', GoTo, self.handle_go_to)
        rospy.Service('/start_trajectory', StartTrajectory, self.handle_start_trajectory)
        rospy.Service('/update_params', UpdateParams, self.handle_update_params)

    def handle_emergency(self, req):
        rospy.logwarn("Emergency not implemented in simulation!")
        return EmptyResponse()

    def handle_takeoff(self, req):
        for cf in self.crazyflies:
            cf.handle_takeoff(req)
        return TakeoffResponse()

    def handle_land(self, req):
        for cf in self.crazyflies:
            cf.handle_land(req)
        return LandResponse()

    def handle_go_to(self, req):
        for cf in self.crazyflies:
            cf.handle_go_to(req)
        return GoToResponse()

    def handle_start_trajectory(self, req):
        for cf in self.crazyflies:
            cf.handle_start_trajectory(req)
        return StartTrajectoryResponse()

    def handle_update_params(self, req):
        for cf in self.crazyflies:
            cf.handle_update_params(req)
        return UpdateParamsResponse()


def main():
    rospy.init_node("CrazyflieROSSim", anonymous=False)
    dt = rospy.get_param('dt', 0.1)

    timeHelper = TimeHelperROS()
    srv = CrazyflieServerROSSim(timeHelper)

    rate = rospy.Rate(1/dt)  # hz

    br = tf2_ros.TransformBroadcaster()
    transform = geometry_msgs.msg.TransformStamped()
    transform.header.frame_id = "world"
    transform.transform.rotation.x = 0
    transform.transform.rotation.y = 0
    transform.transform.rotation.z = 0
    transform.transform.rotation.w = 1
    br.sendTransform(transform)

    while not rospy.is_shutdown():
        transform.header.stamp = rospy.Time.now()
        for cf in srv.crazyflies:
            cf.cfsim.integrate(dt, 0, np.inf)
            if not cf.stopped:
                cfid = cf.cfsim.id
                pos = cf.cfsim.position()
                if np.isfinite(pos).all():
                    transform.child_frame_id = "/cf" + str(cfid)
                    transform.transform.translation.x = pos[0]
                    transform.transform.translation.y = pos[1]
                    transform.transform.translation.z = pos[2]
                    br.sendTransform(transform)
        for cf in srv.crazyflies:
            cf.cfsim.flip()
        rate.sleep()

if __name__ == "__main__":
    main()
