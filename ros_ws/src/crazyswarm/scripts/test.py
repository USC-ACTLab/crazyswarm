#!/usr/bin/env python

import sys
import rospy
from std_srvs.srv import Empty

if __name__ == "__main__":
    rospy.waitForService("/emergency");
    emergency = rospy.ServiceProxy("/emergency", Empty)
    rospy.waitForService("/takeoff");
    takeoff = rospy.ServiceProxy("/takeoff", Empty)
    rospy.waitForService("/land");
    land = rospy.ServiceProxy("/land", Empty)
    rospy.waitForService("/start_trajectory");
    startTrajectory = rospy.ServiceProxy("/start_trajectory", Empty)
    rospy.waitForService("/ellipse");
    startTrajectory = rospy.ServiceProxy("/ellipse", Empty)
