import argparse
import numpy as np
import math
import os

import matplotlib.mlab as mlab
import matplotlib.pyplot as plt

def compare(folder, file):
    fileSim = os.path.join(folder, "sim", file)
    fileReal = os.path.join(folder, "real", file)
    if not os.path.exists(os.path.join(folder, "output")):
        os.mkdir(os.path.join(folder, "output"))
    output = os.path.join(folder, "output") + "/" + os.path.splitext(file)[0] + "_"

    matrix_sim = np.loadtxt(fileSim, delimiter=',', skiprows=1)
    tsim = matrix_sim[:,0]
    possim = matrix_sim[:,1:4]
    matrix_real = np.loadtxt(fileReal, delimiter=',', skiprows=1)
    treal = matrix_real[:,0]
    posreal = matrix_real[:,1:4]

    fig = plt.figure()
    plt.plot(tsim, possim[:,0], label='Simulation')
    plt.plot(treal, posreal[:,0], label='Real')
    plt.legend()
    plt.ylabel('X [m]')
    plt.xlabel('Time [s]')
    plt.savefig(output + "x.svg")

    fig = plt.figure()
    plt.plot(tsim, possim[:,1], label='Simulation')
    plt.plot(treal, posreal[:,1], label='Real')
    plt.legend()
    plt.ylabel('Y [m]')
    plt.xlabel('Time [s]')
    plt.savefig(output + "y.svg")
    plt.close()

    fig = plt.figure()
    plt.plot(tsim, possim[:,2], label='Simulation')
    plt.plot(treal, posreal[:,2], label='Real')
    plt.legend()
    plt.ylabel('Z [m]')
    plt.xlabel('Time [s]')
    plt.savefig(output + "z.svg")
    plt.close()

    # Make sure the data files align roughly
    rows = min(len(tsim), len(treal))
    tsim = tsim[0:rows]
    possim = possim[0:rows]
    treal = treal[0:rows]
    posreal = posreal[0:rows]

    dt = np.fabs(tsim - treal)
    if np.max(dt) > 0.008:
        print("ERROR: Time doesn't align!")
        print(np.max(dt))
    else:
        # all clear => compute errors etc.
        poserror = possim - posreal
        disterror = np.linalg.norm(poserror, axis=1)

        fig = plt.figure()
        plt.plot(tsim, poserror[:,0])
        plt.ylabel('Error [m]')
        plt.xlabel('Time [s]')
        plt.savefig(output + "errorx.svg")
        plt.close()

        fig = plt.figure()
        plt.plot(tsim, poserror[:,1])
        plt.ylabel('Error [m]')
        plt.xlabel('Time [s]')
        plt.savefig(output + "errory.svg")
        plt.close()

        fig = plt.figure()
        plt.plot(tsim, poserror[:,2])
        plt.ylabel('Error [m]')
        plt.xlabel('Time [s]')
        plt.savefig(output + "errorz.svg")
        plt.close()

        fig = plt.figure()
        plt.plot(tsim, disterror)
        plt.ylabel('Error [m]')
        plt.xlabel('Time [s]')
        plt.savefig(output + "errordist.svg")
        plt.close()

        totalError = np.sum(disterror[0:-1] * np.diff(tsim))
        result = totalError / tsim[-1]
        return result

if __name__ == "__main__":
    # Parse input arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", help="folder name to analyzse")
    args = parser.parse_args()

    files = next(os.walk(os.path.join(args.folder, "sim")))[2]
    for file in files:
        result = compare(args.folder, file)
        print("{} Avg. error: {} m".format(file, result))
