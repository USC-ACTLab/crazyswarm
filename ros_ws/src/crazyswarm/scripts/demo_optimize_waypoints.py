#!/usr/bin/env python

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

from crazyflie import *
from trajectory import *

if __name__ == "__main__":
    #allcfs = CrazyflieServer()
    #allcfs.takeoff()
    #time.sleep(2)

    # an L-shaped trajectory in the XY plane.

    x0 = [0, 0, 0, 0] # 0 initial pos thru jerk
    x1 = [1]          # pos only
    x2 = [1, 0, 0, 0] # final pos thru jerk
    x_wp = [x0, x1, x2]

    y0 = [0, 0, 0, 0]
    y1 = [0]
    y2 = [1, 0, 0, 0]
    y_wp = [y0, y1, y2]

    z0 = [0, 0, 0, 0]
    z1 = [0, 0, 0, 0]
    z2 = [0, 0, 0, 0]
    z_wp = [z0, z1, z2]

    yaw0 = [0, 0, 0, 0]
    yaw1 = [0, 0, 0, 0]
    yaw2 = [0, 0, 0, 0]
    yaw_wp = [yaw0, yaw1, yaw2]

    duration = 40

    traj = Trajectory()
    traj.optimize_waypoints(x_wp, y_wp, z_wp, yaw_wp, duration)

    # plot for verification
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(traj.plotdata[0], traj.plotdata[1], traj.plotdata[2])

    #for id, cf in allcfs.crazyflies.iteritems():
        #cf.uploadTrajectory(traj)

    #allcfs.startTrajectory()
    #time.sleep(traj.totalDuration())
    #allcfs.land()
