import argparse
import numpy as np
import math
import os

import matplotlib as mpl
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt

mpl.rcParams['svg.fonttype'] = 'none'

width = 0.5
colors = ["#e66101", "#fdb863", "#b2abd2", "#5e3c99" ]

if __name__ == "__main__":

    numCFs = [1,4,9,16,25,36,49]
    avgsDist = []
    avgsX = []
    avgsY = []
    avgsZ = []
    stdsDist = []
    for num in numCFs:
        matrix = np.loadtxt("{}.csv".format(num), delimiter=',', skiprows=0,ndmin=2)
        # print(matrix)
        avgDist = np.mean(matrix[:,1])
        avgX = np.mean(matrix[:,2])
        avgY = np.mean(matrix[:,3])
        avgZ = np.mean(matrix[:,4])
        stdDist = np.std(matrix[:,1])
        avgsDist.append(avgDist)
        avgsX.append(avgX)
        avgsY.append(avgY)
        avgsZ.append(avgZ)

        stdsDist.append(stdDist)
        # print(avg,std)

    ind = np.arange(len(numCFs))

    fig = plt.figure(figsize=(12/2.0,8/2.0))
    ax = fig.add_subplot(111)

    res = ax.bar(ind, avgsDist, width, color=colors[0], align="center")
    (_, caps, _) = plt.errorbar(ind, avgsDist, yerr=stdsDist, fmt='o', markersize=0, capsize=5, color=colors[3], elinewidth=3)
    for cap in caps:
        cap.set_markeredgewidth(3)

    # res = ax.bar(ind + 0.2, avgsX, width, color=colors[1], align="center")
    # res = ax.bar(ind + 0.4, avgsY, width, color=colors[2], align="center")
    # res = ax.bar(ind + 0.6, avgsZ, width, color=colors[3], align="center")

    ax.set_xticks(ind)
    ax.set_xticklabels(('1', '2x2', '3x3', '4x4', '5x5', '6x6', '7x7'))
    plt.ylabel("Average Euclidean Tracking error [m]")
    plt.xlabel("Crazyflie Formation")

    plt.show()
    # plt.savefig("tracking.png", bbox_inches='tight')
