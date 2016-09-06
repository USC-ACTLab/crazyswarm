import argparse
import numpy as np
import math

import matplotlib.mlab as mlab
import matplotlib.pyplot as plt

lastEvent = None

def onclick(event):
    global lastEvent
    print("t={} ms, y={}".format(event.xdata, event.ydata))
    if lastEvent is not None:
        lastt = lastEvent.xdata
        t = event.xdata
        dt = math.fabs(lastt - t)
        print("dt: {} ms".format(dt))
    lastEvent = event


if __name__ == "__main__":
    # Parse input arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file", help="input ranges csv")
    args = parser.parse_args()

    matrix = np.loadtxt(args.csv_file, delimiter=',', skiprows=1)
    t = matrix[:,0]
    imu = matrix[:,1]
    vicon = matrix[:,2]

    fig = plt.figure()
    cid = fig.canvas.mpl_connect('button_press_event', onclick)
    plt.plot(t, imu, label='imu (rad/s)')
    plt.plot(t, vicon, label='vicon (rad)')
    plt.legend()
    plt.xlabel('Time [ms]')

    plt.show()
