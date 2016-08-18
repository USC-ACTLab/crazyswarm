#!/usr/bin/env python

import sys
import yaml
import rospy
import numpy as np
from std_srvs.srv import Empty
from crazyflie_driver.srv import *
from tf import TransformListener

def arrayToGeometryPoint(a):
    return geometry_msgs.msg.Point(a[0], a[1], a[2])


class Crazyflie:
    def __init__(self, id, initialPosition, tf):
        self.id = id
        prefix = "/cf" + str(id)
        self.initialPosition = np.array(initialPosition)
        rospy.wait_for_service(prefix + "/upload_trajectory");
        self.uploadTrajectoryService = rospy.ServiceProxy(prefix + "/upload_trajectory", UploadTrajectory)
        rospy.wait_for_service(prefix + "/set_ellipse");
        self.setEllipseService = rospy.ServiceProxy(prefix + "/set_ellipse", SetEllipse)
        rospy.wait_for_service(prefix + "/takeoff")
        self.takeoffService = rospy.ServiceProxy(prefix + "/takeoff", Takeoff)
        rospy.wait_for_service(prefix + "/land")
        self.landService = rospy.ServiceProxy(prefix + "/land", Land)
        rospy.wait_for_service(prefix + "/hover")
        self.hoverService = rospy.ServiceProxy(prefix + "/hover", Hover)
        rospy.wait_for_service(prefix + "/avoid_target")
        self.avoidTargetService = rospy.ServiceProxy(prefix + "/avoid_target", AvoidTarget)
        rospy.wait_for_service(prefix + "/set_group")
        self.setGroupService = rospy.ServiceProxy(prefix + "/set_group", SetGroup)
        self.tf = tf

    def uploadTrajectory(self, trajectory):
        # request = UploadTrajectory()
        # request.polygons = trajectory.polygons
        self.uploadTrajectoryService(trajectory.polygons)

    def setEllipse(self, center, major, minor, period):
        self.setEllipseService(
            arrayToGeometryPoint(center),
            arrayToGeometryPoint(major),
            arrayToGeometryPoint(minor),
            rospy.Duration.from_sec(period))

    def takeoff(self, targetHeight, duration):
        self.takeoffService(0, targetHeight, rospy.Duration.from_sec(duration))

    def land(self, targetHeight, duration):
        self.landService(0, targetHeight, rospy.Duration.from_sec(duration))

    def hover(self, goal, yaw, duration):
        gp = arrayToGeometryPoint(goal)
        self.hoverService(gp, yaw, rospy.Duration.from_sec(duration))

    def avoidTarget(self, home, maxDisplacement, maxSpeed):
        home = arrayToGeometryPoint(home)
        self.avoidTargetService(home, maxDisplacement, maxSpeed)

    def setGroup(self, group):
        self.setGroupService(group)

    def position(self):
        self.tf.waitForTransform("/world", "/cf" + str(self.id), rospy.Time(0), rospy.Duration(10))
        position, quaternion = self.tf.lookupTransform("/world", "/cf" + str(self.id), rospy.Time(0))
        # if self.tf.frameExists("/cf" + self.id) and self.tf.frameExists("/world"):
            # t = self.tf.getLatestCommonTime("/cf" + self.id, "/world")
            # position, quaternion = self.tf.lookupTransform("/cf" + self.id, "/world", t)
        return np.array(position)

class CrazyflieServer:
    def __init__(self):
        rospy.init_node("CrazyflieAPI", anonymous=False)
        rospy.wait_for_service("/emergency")
        self.emergencyService = rospy.ServiceProxy("/emergency", Empty)
        rospy.wait_for_service("/takeoff")
        self.takeoffService = rospy.ServiceProxy("/takeoff", Takeoff)
        rospy.wait_for_service("/land")
        self.landService = rospy.ServiceProxy("/land", Land)
        rospy.wait_for_service("/start_trajectory");
        self.startTrajectoryService = rospy.ServiceProxy("/start_trajectory", Empty)
        rospy.wait_for_service("/start_ellipse")
        self.ellipseService = rospy.ServiceProxy("/start_ellipse", Empty)
        rospy.wait_for_service("/start_canned_trajectory")
        self.startCannedTrajectoryService = rospy.ServiceProxy("/start_canned_trajectory", StartCannedTrajectory)
        rospy.wait_for_service("/go_home");
        self.goHomeService = rospy.ServiceProxy("/go_home", Empty)

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

    def takeoff(self, targetHeight, duration, group = 0):
        self.takeoffService(group, targetHeight, rospy.Duration.from_sec(duration))

    def land(self, targetHeight, duration, group = 0):
        self.landService(group, targetHeight, rospy.Duration.from_sec(duration))

    def startTrajectory(self, group = 0):
        self.startTrajectoryService(group)

    def startEllipse(self, group = 0):
        self.ellipseService(group)

    def startCannedTrajectory(self, trajectory, timescale, group = 0):
        self.startCannedTrajectoryService(group, trajectory, timescale)

    def goHome(self, group = 0):
        self.goHomeService(group)
