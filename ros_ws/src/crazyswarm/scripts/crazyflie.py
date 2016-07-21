#!/usr/bin/env python

import sys
import yaml
import rospy
from std_srvs.srv import Empty
from crazyflie_driver.srv import *

def arrayToGeometryPoint(a):
    return geometry_msgs.msg.Point(a[0], a[1], a[2])


class Crazyflie:
    def __init__(self, id):
        self.id = id
        rospy.wait_for_service("/cf" + id + "/upload_trajectory");
        self.uploadTrajectoryService = rospy.ServiceProxy("/cf" + id + "/upload_trajectory", UploadTrajectory)
        rospy.wait_for_service("/cf" + id + "/set_ellipse");
        self.setEllipseService = rospy.ServiceProxy("/cf" + id + "/set_ellipse", SetEllipse)

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

class CrazyflieServer:
    def __init__(self):
        rospy.init_node("CrazyflieAPI", anonymous=False)
        rospy.wait_for_service("/emergency")
        self.emergencyService = rospy.ServiceProxy("/emergency", Empty)
        rospy.wait_for_service("/takeoff")
        self.takeoffService = rospy.ServiceProxy("/takeoff", Empty)
        rospy.wait_for_service("/land")
        self.landService = rospy.ServiceProxy("/land", Empty)
        rospy.wait_for_service("/start_trajectory");
        self.startTrajectoryService = rospy.ServiceProxy("/start_trajectory", Empty)
        rospy.wait_for_service("/start_ellipse")
        self.ellipseService = rospy.ServiceProxy("/start_ellipse", Empty)

        with open("../launch/crazyflies.yaml", 'r') as ymlfile:
            cfg = yaml.load(ymlfile)

        self.crazyflies = dict()
        for crazyflie in cfg["crazyflies"]:
            id = crazyflie["id"]
            self.crazyflies[id] = Crazyflie(id)

    def emergency(self):
        self.emergencyService()

    def takeoff(self):
        self.takeoffService()

    def land(self):
        self.landService()

    def startTrajectory(self):
        self.startTrajectoryService()

    def startEllipse(self):
        self.ellipseService()





