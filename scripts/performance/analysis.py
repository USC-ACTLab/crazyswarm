import argparse
import numpy as np
import math

import matplotlib.mlab as mlab
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # Parse input arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file_sim", help="input csv1")
    parser.add_argument("csv_file_real", help="input csv2")
    args = parser.parse_args()

    matrix_sim = np.loadtxt(args.csv_file_sim, delimiter=',', skiprows=1)
    tsim = matrix_sim[:,0]
    possim = matrix_sim[:,1:4]
    matrix_real = np.loadtxt(args.csv_file_real, delimiter=',', skiprows=1)
    treal = matrix_real[:,0]
    posreal = matrix_real[:,1:4]

    fig = plt.figure()
    plt.plot(tsim, possim[:,0], label='Simulation')
    plt.plot(treal, posreal[:,0], label='Real')
    plt.legend()
    plt.ylabel('X [m]')
    plt.xlabel('Time [s]')
    plt.savefig("x.svg")

    fig = plt.figure()
    plt.plot(tsim, possim[:,1], label='Simulation')
    plt.plot(treal, posreal[:,1], label='Real')
    plt.legend()
    plt.ylabel('Y [m]')
    plt.xlabel('Time [s]')
    plt.savefig("y.svg")

    fig = plt.figure()
    plt.plot(tsim, possim[:,2], label='Simulation')
    plt.plot(treal, posreal[:,2], label='Real')
    plt.legend()
    plt.ylabel('Z [m]')
    plt.xlabel('Time [s]')
    plt.savefig("z.svg")

    # Make sure the data files align roughly
    rows = min(len(tsim), len(treal))
    tsim = tsim[0:rows]
    possim = possim[0:rows]
    treal = treal[0:rows]
    posreal = posreal[0:rows]

    dt = np.fabs(tsim - treal)
    if np.max(dt) > 0.005:
        print("ERROR: Time doesn't align!")
    else:
        # all clear => compute errors etc.
        poserror = possim - posreal
        disterror = np.linalg.norm(poserror, axis=1)

        fig = plt.figure()
        plt.plot(tsim, poserror[:,0])
        plt.ylabel('Error [m]')
        plt.xlabel('Time [s]')
        plt.savefig("errorx.svg")

        fig = plt.figure()
        plt.plot(tsim, poserror[:,1])
        plt.ylabel('Error [m]')
        plt.xlabel('Time [s]')
        plt.savefig("errory.svg")

        fig = plt.figure()
        plt.plot(tsim, poserror[:,2])
        plt.ylabel('Error [m]')
        plt.xlabel('Time [s]')
        plt.savefig("errorz.svg")

        fig = plt.figure()
        plt.plot(tsim, disterror)
        plt.ylabel('Error [m]')
        plt.xlabel('Time [s]')
        plt.savefig("errordist.svg")

        totalError = np.sum(disterror[0:-1] * np.diff(tsim))
        print("Avg. error: {} m".format(totalError / tsim[-1]))
