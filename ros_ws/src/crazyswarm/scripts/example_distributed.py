#!/usr/bin/env python3

import rospy
from tf import TransformListener

import yaml
import numpy as np
from pycrazyswarm.crazyflie import Crazyflie


if __name__ == "__main__":

    rospy.init_node("CrazyflieDistributed", anonymous=True)

    with open(rospy.get_param("crazyflies_yaml"), 'r') as ymlfile:
        cfg = yaml.safe_load(ymlfile)

    tf = TransformListener()

    cf = None
    for crazyflie in cfg["crazyflies"]:
        cfid = int(crazyflie["id"])
        if cfid == 1:
            initialPosition = crazyflie["initialPosition"]
            cf = Crazyflie(cfid, initialPosition, tf)
            break

    if cf is None:
        exit("No CF with required ID found!")

    cf.takeoff(0.5, 2.0)
    rospy.sleep(2.5)
    cf.land(0.02, 2.0)
