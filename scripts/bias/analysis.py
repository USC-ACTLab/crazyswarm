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
    print(np.mean(matrix[:,1]), np.std(matrix[:,1]))
    print(np.mean(matrix[:,2]), np.std(matrix[:,2]))
    print(np.mean(matrix[:,3]), np.std(matrix[:,3]))
    print(np.mean(matrix[:,4]), np.std(matrix[:,4]))
    print(np.mean(matrix[:,5]), np.std(matrix[:,5]))
    print(np.mean(matrix[:,6]), np.std(matrix[:,6]))
