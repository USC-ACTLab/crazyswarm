#!/usr/bin/env python


import sys
import yaml
import rospy
import numpy as np
import time
import tf_conversions
from std_srvs.srv import Empty
import std_msgs
from crazyflie_driver.srv import *
from crazyflie_driver.msg import TrajectoryPolynomialPiece, FullState
from tf import TransformListener

def arrayToGeometryPoint(a):
    return geometry_msgs.msg.Point(a[0], a[1], a[2])

class TimeHelper:
    def __init__(self):
        pass

    def time(self):
        return time.time()

    def sleep(self, duration):
        time.sleep(duration)


class Crazyflie:
    def __init__(self, id, initialPosition, tf):
        self.id = id
        prefix = "/cf" + str(id)
        self.prefix = prefix
        self.initialPosition = np.array(initialPosition)

        self.tf = tf

        rospy.wait_for_service(prefix + "/set_group_mask")
        self.setGroupMaskService = rospy.ServiceProxy(prefix + "/set_group_mask", SetGroupMask)
        rospy.wait_for_service(prefix + "/takeoff")
        self.takeoffService = rospy.ServiceProxy(prefix + "/takeoff", Takeoff)
        rospy.wait_for_service(prefix + "/land")
        self.landService = rospy.ServiceProxy(prefix + "/land", Land)
        # rospy.wait_for_service(prefix + "/stop")
        # self.stopService = rospy.ServiceProxy(prefix + "/stop", Stop)
        rospy.wait_for_service(prefix + "/go_to")
        self.goToService = rospy.ServiceProxy(prefix + "/go_to", GoTo)
        rospy.wait_for_service(prefix + "/upload_trajectory")
        self.uploadTrajectoryService = rospy.ServiceProxy(prefix + "/upload_trajectory", UploadTrajectory)
        rospy.wait_for_service(prefix + "/start_trajectory")
        self.startTrajectoryService = rospy.ServiceProxy(prefix + "/start_trajectory", StartTrajectory)
        rospy.wait_for_service(prefix + "/update_params")
        self.updateParamsService = rospy.ServiceProxy(prefix + "/update_params", UpdateParams)

        self.cmdFullStatePublisher = rospy.Publisher(prefix + "/cmd_full_state", FullState, queue_size=1)
        self.cmdFullStateMsg = FullState()
        self.cmdFullStateMsg.header.seq = 0
        self.cmdFullStateMsg.header.frame_id = "/world"

        self.cmdStopPublisher = rospy.Publisher(prefix + "/cmd_stop", std_msgs.msg.Empty, queue_size=1)

        self.cmdVelPublisher = rospy.Publisher(prefix + "/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)

    def setGroupMask(self, groupMask):
        self.setGroupMaskService(groupMask)

    def takeoff(self, targetHeight, duration, groupMask = 0):
        self.takeoffService(groupMask, targetHeight, rospy.Duration.from_sec(duration))

    def land(self, targetHeight, duration, groupMask = 0):
        self.landService(groupMask, targetHeight, rospy.Duration.from_sec(duration))

    def stop(self, groupMask = 0):
        self.stopService(groupMask)

    def goTo(self, goal, yaw, duration, relative = False, groupMask = 0):
        gp = arrayToGeometryPoint(goal)
        self.goToService(groupMask, relative, gp, yaw, rospy.Duration.from_sec(duration))

    def uploadTrajectory(self, trajectoryId, pieceOffset, trajectory):
        pieces = []
        for poly in trajectory.polynomials:
            piece = TrajectoryPolynomialPiece()
            piece.duration = rospy.Duration.from_sec(poly.duration)
            piece.poly_x   = poly.px.p
            piece.poly_y   = poly.py.p
            piece.poly_z   = poly.pz.p
            piece.poly_yaw = poly.pyaw.p
            pieces.append(piece)
        self.uploadTrajectoryService(trajectoryId, pieceOffset, pieces)

    def startTrajectory(self, trajectoryId, timescale = 1.0, reverse = False, relative = True, groupMask = 0):
        self.startTrajectoryService(groupMask, trajectoryId, timescale, reverse, relative)

    def position(self):
        self.tf.waitForTransform("/world", "/cf" + str(self.id), rospy.Time(0), rospy.Duration(10))
        position, quaternion = self.tf.lookupTransform("/world", "/cf" + str(self.id), rospy.Time(0))
        return np.array(position)

    def getParam(self, name):
        return rospy.get_param(self.prefix + "/" + name)

    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.updateParamsService([name])

    def setParams(self, params):
        for name, value in params.iteritems():
            rospy.set_param(self.prefix + "/" + name, value)
        self.updateParamsService(params.keys())

    def cmdFullState(self, pos, vel, acc, yaw, omega):
        self.cmdFullStateMsg.header.stamp = rospy.Time.now()
        self.cmdFullStateMsg.header.seq += 1
        self.cmdFullStateMsg.pose.position.x    = pos[0]
        self.cmdFullStateMsg.pose.position.y    = pos[1]
        self.cmdFullStateMsg.pose.position.z    = pos[2]
        self.cmdFullStateMsg.twist.linear.x     = vel[0]
        self.cmdFullStateMsg.twist.linear.y     = vel[1]
        self.cmdFullStateMsg.twist.linear.z     = vel[2]
        self.cmdFullStateMsg.acc.x              = acc[0]
        self.cmdFullStateMsg.acc.y              = acc[1]
        self.cmdFullStateMsg.acc.z              = acc[2]
        self.cmdFullStateMsg.pose.orientation   = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))
        self.cmdFullStateMsg.twist.angular.x    = omega[0]
        self.cmdFullStateMsg.twist.angular.y    = omega[1]
        self.cmdFullStateMsg.twist.angular.z    = omega[2]
        self.cmdFullStatePublisher.publish(self.cmdFullStateMsg)

    def cmdStop(self):
        self.cmdStopPublisher.publish(std_msgs.msg.Empty())

    def cmdVel(self, roll, pitch, yawrate, thrust):
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = pitch
        msg.linear.y = roll
        msg.angular.z = yawrate
        msg.linear.z = thrust
        self.cmdVelPublisher.publish(msg)

    #
    # wrappers around the parameter setting system for common cases
    #

    # - this is a blocking command, so it may cause stability problems
    #   for large swarms and/or high-frequency changes
    # - parameter ring/effect must be set to 7 (solid color) to have any effect!
    # - input should be floats in range [0, 1]
    def setLEDColor(self, r, g, b):
        self.setParam("ring/solidRed", int(r * 255))
        self.setParam("ring/solidGreen", int(g * 255))
        self.setParam("ring/solidBlue", int(b * 255))


class CrazyflieServer:
    def __init__(self):
        rospy.init_node("CrazyflieAPI", anonymous=False)
        rospy.wait_for_service("/emergency")
        self.emergencyService = rospy.ServiceProxy("/emergency", Empty)
        rospy.wait_for_service("/takeoff")
        self.takeoffService = rospy.ServiceProxy("/takeoff", Takeoff)
        rospy.wait_for_service("/land")
        self.landService = rospy.ServiceProxy("/land", Land)
        # rospy.wait_for_service("/stop")
        # self.stopService = rospy.ServiceProxy("/stop", Stop)
        rospy.wait_for_service("/go_to")
        self.goToService = rospy.ServiceProxy("/go_to", GoTo)
        rospy.wait_for_service("/start_trajectory");
        self.startTrajectoryService = rospy.ServiceProxy("/start_trajectory", StartTrajectory)
        rospy.wait_for_service("/update_params")
        self.updateParamsService = rospy.ServiceProxy("/update_params", UpdateParams)

        with open("../launch/crazyflies.yaml", 'r') as ymlfile:
            cfg = yaml.load(ymlfile)

        self.tf = TransformListener()

        self.crazyflies = []
        self.crazyfliesById = dict()
        for crazyflie in cfg["crazyflies"]:
            id = int(crazyflie["id"])
            initialPosition = crazyflie["initialPosition"]
            cf = Crazyflie(id, initialPosition, self.tf)
            self.crazyflies.append(cf)
            self.crazyfliesById[id] = cf

    def emergency(self):
        self.emergencyService()

    def takeoff(self, targetHeight, duration, groupMask = 0):
        self.takeoffService(groupMask, targetHeight, rospy.Duration.from_sec(duration))

    def land(self, targetHeight, duration, groupMask = 0):
        self.landService(groupMask, targetHeight, rospy.Duration.from_sec(duration))

    # def stop(self, groupMask = 0):
    #     self.stopService(groupMask)

    def goTo(self, goal, yaw, duration, groupMask = 0):
        gp = arrayToGeometryPoint(goal)
        self.goToService(groupMask, True, gp, yaw, rospy.Duration.from_sec(duration))

    def startTrajectory(self, trajectoryId, timescale = 1.0, reverse = False, relative = True, groupMask = 0):
        self.startTrajectoryService(groupMask, trajectoryId, timescale, reverse, relative)

    def setParam(self, name, value):
        rospy.set_param("/allcfs/" + name, value)
        self.updateParamsService([name])
