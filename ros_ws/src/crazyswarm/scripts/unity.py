#!/usr/bin/env python

import numpy as np
import socket
import rospy
from pycrazyswarm import *

BUFF_SIZE = 1024
PORT = 6112
RATE = 10
SPEED = 1.5 # m/s
CRAZYFLIE_IDS = [48, 49]
PRINT_ONLY = False

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    #network stuff
    Socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    Socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    Socket.bind(('',PORT))
    print("** Listening on '{0}'".format(PORT))
    Socket.listen(1)
    #Socket.setblocking(0)
    #Socket.settimeout(None) #1.0/RATE)
    Socket.settimeout(1.0/RATE)
    rate = rospy.Rate(RATE)

    while not rospy.is_shutdown():
        try:
            connection, address = Socket.accept()
            print("** Connection from '{0}' on '{1}'".format(address, PORT))
            break
        except:
            pass

    print("Continuing")

    # execute waypoints

    if not PRINT_ONLY:
        allcfs.takeoff(targetHeight=0.85, duration=2.0)
        timeHelper.sleep(2.0)

    dataStr = ""
    lastUpdate = rospy.get_time()
    while not rospy.is_shutdown():
        lastData = connection.recv(BUFF_SIZE)
        t = rospy.get_time()
        if t - lastUpdate >= 1.0/RATE:
            cfs = lastData[0:-1].split(';')
            if len(cfs) == len(CRAZYFLIE_IDS):  
                for idx in range(0, len(cfs)): 
                    cf = cfs[idx]
                    cf_id = CRAZYFLIE_IDS[idx]         
                    xyzyaw = cf.split(',')
                    if len(xyzyaw) == 4:
                        x,y,z,yaw = xyzyaw
                        cf = allcfs.crazyfliesById[cf_id]
                        pos = cf.position()
                        goalPos = np.array([-float(x),-float(y),float(z)])
                        distance = np.linalg.norm(pos - goalPos)
                        timeToGoal = max(distance / SPEED, 1.0)                
                        print(cf_id, goalPos,yaw,timeToGoal,distance)
                        if not PRINT_ONLY:
                            cf.hover(goalPos, float(yaw), timeToGoal)
                        lastUpdate = t

#        while True:
#            newDataStr = connection.recv(BUFF_SIZE)
#            print(newDataStr)
#            if newDataStr:
#                dataStr += newDataStr
#            else:
#                break
#        dataStr = dataStr.rstrip('!')
#        if len(dataStr.split('!')) > 0:
#            data = dataStr.split('!')
#            if len(data) > 2:
#                xyzyaw = data[len(data)-2].split(',')
#                if len(xyzyaw) == 4:
#                    x,y,z,yaw = xyzyaw
#                    cf = allcfs.crazyfliesById[231]
#                    pos = cf.position()
#                    goalPos = np.array([float(x),float(y),float(z)])
#                    distance = np.linalg.norm(pos - goalPos)
#                    t = distance / SPEED                
#                    print(x,y,z,yaw,t,distance)
#                    #cf.hover(goalPos, 0, t) #float(yaw), t)                
#                    dataStr = ""
#                else:
#                    print("Malformed data!", xyzyaw, dataStr)
#
#        rate.sleep()

