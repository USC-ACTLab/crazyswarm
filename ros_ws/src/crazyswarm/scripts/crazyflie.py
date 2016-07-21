#!/usr/bin/env python

import sys
import yaml
import rospy
from std_srvs.srv import Empty
from crazyflie_driver.srv import UploadTrajectory


class Crazyflie:
    def __init__(self, id):
        self.id = id
        rospy.waitForService("/cf" + id + "/upload_trajectory");
        self.uploadTrajectoryService = rospy.ServiceProxy("/cf" + id + "/upload_trajectory", UploadTrajectory)

    def uploadTrajectory(self, trajectory):
        request = UploadTrajectory()
        request.polygons = trajectory.polygons
        uploadTrajectoryService(request)

class CrazyflieServer:
    def __init__(self):
        rospy.init_node("CrazyflieAPI", anonymous=False)
        rospy.waitForService("/emergency");
        self.emergencyService = rospy.ServiceProxy("/emergency", Empty)
        rospy.waitForService("/takeoff");
        self.takeoffService = rospy.ServiceProxy("/takeoff", Empty)
        rospy.waitForService("/land");
        self.landService = rospy.ServiceProxy("/land", Empty)
        rospy.waitForService("/start_trajectory");
        self.startTrajectoryService = rospy.ServiceProxy("/start_trajectory", Empty)
        rospy.waitForService("/ellipse");
        self.ellipseService = rospy.ServiceProxy("/ellipse", Empty)

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

    def ellipse(self):
        self.ellipseService()





