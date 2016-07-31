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
        self.initialPosition = np.array(initialPosition)
        rospy.wait_for_service("/cf" + id + "/upload_trajectory");
        self.uploadTrajectoryService = rospy.ServiceProxy("/cf" + id + "/upload_trajectory", UploadTrajectory)
        rospy.wait_for_service("/cf" + id + "/set_ellipse");
        self.setEllipseService = rospy.ServiceProxy("/cf" + id + "/set_ellipse", SetEllipse)
        rospy.wait_for_service("/cf" + id + "/takeoff")
        self.takeoffService = rospy.ServiceProxy("/cf" + id + "/takeoff", Takeoff)
        rospy.wait_for_service("/cf" + id + "/land")
        self.landService = rospy.ServiceProxy("/cf" + id + "/land", Land)
        rospy.wait_for_service("/cf" + id + "/hover")
        self.hoverService = rospy.ServiceProxy("/cf" + id + "/hover", Hover)
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
        self.takeoffService(targetHeight, rospy.Duration.from_sec(duration))

    def land(self, targetHeight, duration):
        self.landService(targetHeight, rospy.Duration.from_sec(duration))

    def hover(self, goal, yaw, duration):
        gp = arrayToGeometryPoint(goal)
        self.hoverService(gp, yaw, rospy.Duration.from_sec(duration))

    def position(self):
        self.tf.waitForTransform("/world", "/cf" + self.id, rospy.Time(0), rospy.Duration(10))
        position, quaternion = self.tf.lookupTransform("/world", "/cf" + self.id, rospy.Time(0))
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
        for crazyflie in cfg["crazyflies"]:
            id = str(crazyflie["id"])
            initialPosition = crazyflie["initialPosition"]
            self.crazyflies.append(Crazyflie(id, initialPosition, self.tf))

    def emergency(self):
        self.emergencyService()

    def takeoff(self, targetHeight, duration):
        self.takeoffService(targetHeight, rospy.Duration.from_sec(duration))

    def land(self, targetHeight, duration):
        self.landService(targetHeight, rospy.Duration.from_sec(duration))

    def startTrajectory(self):
        self.startTrajectoryService()

    def startEllipse(self):
        self.ellipseService()

    def startCannedTrajectory(self, trajectory, timescale):
        self.startCannedTrajectoryService(trajectory, timescale)

    def goHome(self):
        self.goHomeService()
