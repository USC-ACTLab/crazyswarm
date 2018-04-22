#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.gridspec as gridspec
import argparse

from pycrazyswarm import *
import pycrazyswarm.cfsim.cffirmware as firm

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("trajectory", type=str, help="CSV file containing trajectory")
  args = parser.parse_args()

  traj = piecewise.loadcsv(args.trajectory)
  mass = 0.033 # kg

  duration = firm.piecewise_duration(traj)

  ts = np.arange(0, duration, 0.1)
  evals = np.empty((len(ts), 9))
  for t, i in zip(ts, range(0, len(ts))):
    e = firm.piecewise_eval(traj, t, mass)
    evals[i, 0] = e.pos.x
    evals[i, 1] = e.pos.y
    evals[i, 2] = e.pos.z
    evals[i, 3] = e.vel.x
    evals[i, 4] = e.vel.y
    evals[i, 5] = e.vel.z
    evals[i, 6] = e.acc.x
    evals[i, 7] = e.acc.y
    evals[i, 8] = e.acc.z

  # Create 3x1 sub plots
  gs = gridspec.GridSpec(3, 1)
  fig = plt.figure()

  ax = plt.subplot(gs[0, 0], projection='3d') # row 0
  ax.plot(evals[:,0], evals[:,1], evals[:,2])

  ax = plt.subplot(gs[1, 0]) # row 1
  ax.plot(ts, np.linalg.norm(evals[:,3:6], axis=1))
  ax.set_ylabel("velocity [m/s]")

  ax = plt.subplot(gs[2, 0]) # row 2
  ax.plot(ts, np.linalg.norm(evals[:,6:9], axis=1))
  ax.set_ylabel("acceleration [m/s^2]")

  plt.show()
