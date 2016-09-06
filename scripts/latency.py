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
    dt = np.diff(t)
    t = matrix[1:,0]
    dThetaImu = matrix[1:,1]
    thetaImu = np.cumsum(dThetaImu * (dt / 1000.0))
    thetaVicon = matrix[1:,2] - np.mean(matrix[0:100,2])

    currentIdx = 0
    latencies = np.array([])

    while True:

        result = np.where(dThetaImu[currentIdx:] > 1)
        if (len(result[0]) == 0):
            break

        startIdx = result[0][0] - 100 + currentIdx
        print(startIdx)
        endIdx = startIdx + 300


        thetaImuSegment = thetaImu[startIdx:endIdx]

        minError = float("inf")
        minLatency = None
        for latency in range(0, 60):
            thetaViconTemp = thetaVicon[startIdx + latency:endIdx+latency]
            error = np.sum(np.abs(thetaImuSegment - thetaViconTemp))
            if error < minError:
                minError = error
                minLatency = latency

        latency = t[startIdx + minLatency] - t[startIdx]
        print("Latency: {}".format(latency))
        latencies = np.append(latencies, latency)

        tSegment = t[startIdx:endIdx]
        thetaViconSegment = thetaVicon[startIdx + minLatency:endIdx+minLatency]

        fig = plt.figure()
        cid = fig.canvas.mpl_connect('button_press_event', onclick)
        plt.plot(tSegment, thetaImuSegment, label='imu (rad)')
        plt.plot(tSegment, thetaViconSegment, label='vicon (rad)')
        plt.legend()
        plt.xlabel('Time [ms]')

        plt.show()

        currentIdx = endIdx

    print("Min: {} ms, Avg: {} ms, Max: {} ms".format(np.min(latencies), np.mean(latencies), np.max(latencies)))
