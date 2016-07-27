#!/usr/bin/env python

import time
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from trajectory import *

if __name__ == "__main__":
    traj = Trajectory()
    traj.load("../launch/figure8_smooth.csv")
    oldPlot = None
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-5,5])
    ax.set_ylim([-5,5])
    ax.set_zlim([0,3])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    for t in np.arange(0, traj.totalDuration(), 0.1):
        x, y, z = traj.evaluate(t)
        if oldPlot is not None:
            ax.collections.remove(oldPlot)
        oldPlot = ax.scatter([x, x], [y, y + 1.0], [z + 1.0, z + 1.0])

        plt.pause(0.001)
        print(t)



    # plt.show()


    # oldcol = wframe

    # Z = generate(X, Y, phi)
    # wframe = ax.plot_wireframe(X, Y, Z, rstride=2, cstride=2)

    # # Remove old line collection before drawing
    # if oldcol is not None:
    #     ax.collections.remove(oldcol)

    # plt.pause(.001)
