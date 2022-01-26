#!/usr/bin/env python3

import rospy
from tf import TransformListener
import numpy as np

from pycrazyswarm.crazyflie import Crazyflie

def run(tf, cf):
    # Advanced users: Use the following to get state information of neighbors
    # position, quaternion = tf.lookupTransform("/world", "/cf" + str(cfid), rospy.Time(0))
    pos = cf.initialPosition.copy()
    cf.takeoff(targetHeight=0.5, duration=2.0)
    for _ in range(10):
        pos[2] = np.random.uniform(0.5, 1.0)
        duration = np.random.uniform(0.5, 2.0)
        rospy.loginfo("CF {} going to {} in {}s".format(cf.id, pos, duration))
        cf.goTo(pos, 0, duration)
        rospy.sleep(duration)

    cf.land(targetHeight=0.02, duration=2.0)

if __name__ == "__main__":

    rospy.init_node("CrazyflieDistributed", anonymous=True)
    cfid = rospy.get_param("~cfid")
    cf = None
    for crazyflie in rospy.get_param("crazyflies"):
        if cfid == int(crazyflie["id"]):
            initialPosition = crazyflie["initialPosition"]
            tf = TransformListener()
            cf = Crazyflie(cfid, initialPosition, tf)
            break

    if cf is None:
        rospy.logwarn("No CF with required ID {} found!".format(cfid))
    else:
        run(tf, cf)
