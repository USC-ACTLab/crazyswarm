import argparse
import numpy as np
import math
import os

import matplotlib.mlab as mlab
import matplotlib.pyplot as plt


if __name__ == "__main__":
    # Parse input arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file", help="csv_file")
    args = parser.parse_args()

    matrix = np.loadtxt(args.csv_file, delimiter=',', skiprows=1)

    fig, ax = plt.subplots()

    numcfs = matrix[:,0]
    vicon = matrix[:,1] + matrix[:,2] + matrix[:,3] + matrix[:,4]
    tracking = np.maximum.reduce([matrix[:,6], matrix[:,8], matrix[:,10]])
    comm = np.maximum.reduce([matrix[:,7], matrix[:,9], matrix[:,11]])
    ax.bar(numcfs, vicon, color='b', label="VICON")
    ax.bar(numcfs, tracking, bottom=vicon, color='r', label="Tracking")
    ax.bar(numcfs, comm, bottom=vicon+tracking, color='g', label="Communication")
    plt.legend()

    plt.show()

