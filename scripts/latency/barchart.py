import argparse
import numpy as np
import math
import os

import matplotlib as mpl
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt

mpl.rcParams['svg.fonttype'] = 'none'

WIDTH = 3.6
colors = ["#e66101", "#fdb863", "#b2abd2", "#5e3c99" ]


if __name__ == "__main__":

    matrix = np.loadtxt("estimatedForPlot.csv", delimiter=',', skiprows=1)

    fig = plt.figure(figsize=(12/2.0,8/2.0))
    ax = fig.add_subplot(111)
    # fig, ax = plt.subplots()

    numcfs = matrix[:,0]
    vicon = matrix[:,1] + matrix[:,2] + matrix[:,3] + matrix[:,4]
    tracking = np.maximum.reduce([matrix[:,6], matrix[:,8], matrix[:,10]])
    comm = np.maximum.reduce([matrix[:,7], matrix[:,9], matrix[:,11]])

    b1 = ax.bar(numcfs, vicon, WIDTH, align="center", color=colors[0], label="VICON")

    b2 = ax.bar(numcfs - 1.2, matrix[:,6], WIDTH/3, align="center", bottom=vicon, color=colors[1], label="Tracking")
    b3 = ax.bar(numcfs - 1.2, matrix[:,7], WIDTH/3, align="center", bottom=vicon+matrix[:,6], color=colors[2], label="Communication")

    ax.bar(numcfs, matrix[:,8], WIDTH/3, align="center", bottom=vicon, color=colors[1], label="Tracking")
    ax.bar(numcfs, matrix[:,9], WIDTH/3, align="center", bottom=vicon+matrix[:,8], color=colors[2], label="Communication")

    ax.bar(numcfs + 1.2, matrix[:,10], WIDTH/3, align="center", bottom=vicon, color=colors[1], label="Tracking")
    ax.bar(numcfs + 1.2, matrix[:,11], WIDTH/3, align="center", bottom=vicon+matrix[:,10], color=colors[2], label="Communication")

    plt.ylabel("Latency [ms]")
    plt.xlabel("Number of Crazyflies")
    plt.legend((b3, b2, b1), ("Communication (3 radios)", "Tracking (3 threads)", "VICON"), loc="upper left", frameon=False)
    plt.xlim(-2,52)

    # plt.show()
    plt.savefig("latency.png", bbox_inches='tight')
