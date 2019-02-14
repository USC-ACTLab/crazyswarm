#!/usr/bin/env python

import argparse
import os
import os.path

import numpy as np

from pycrazyswarm import *
import uav_trajectory


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("path",
        help="directory containing numbered subdirectories for each robot," +
            "each of which contains numbered <n>.csv files for each formation change")
    args, unknown = parser.parse_known_args()

    # load trajectory sequences
    root = args.path
    robot_dirs = sorted(os.listdir(root), key=int)
    seqs = [load_all_csvs(os.path.join(root, d)) for d in robot_dirs]
    N = len(robot_dirs)
    steps = len(seqs[0])

    # validate sequences w.r.t. each other
    assert all(len(seq) == steps for seq in seqs)
    for i in range(steps):
        agent_lens = [seq[i].duration for seq in seqs]
        assert all(agent_lens == agent_lens[0])
    step_lens = [t.duration for t in seqs[0]]

    # initialize crazyswarm
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    crazyflies = allcfs.crazyflies
    assert len(crazyflies) == N

    # check that crazyflies.yaml initial positions match sequences.
    # only compare xy positions.
    # assume that cf id numerical order matches sequence order,
    # but id's don't need to be 1..N.
    crazyflies = sorted(crazyflies, key=lambda cf: cf.id)
    init_positions = np.stack([cf.initialPosition for cf in crazyflies])
    evals = [seq[0].eval(0.0).pos for seq in seqs]
    traj_starts = np.stack(evals)
    errs = init_positions - traj_starts
    errnorms = np.linalg.norm(errs[:,:2], axis=1)
    assert not np.any(np.abs(errnorms) > 0.1)

    # upload the trajectories
    for cf, seq in zip(crazyflies, seqs):
        for i, traj in enumerate(seq):
            cf.uploadTrajectory(i, 0, traj)

    # take off - get height from trajectories - must be all same
    z_init = traj_starts[0,2]
    assert np.all(traj_starts[:,2] == z_init)
    t_takeoff = max(2.0, 2.0 * z_init)
    allcfs.takeoff(targetHeight=z_init, duration=t_takeoff)
    timeHelper.sleep(t_takeoff + 1.0)

    # execute the trajectory sequence
    timescale = 0.3
    pause_between = 2.0
    for step in range(steps):
        allcfs.startTrajectory(step, timescale=timescale)
        timeHelper.sleep(step_lens[step] * timescale + pause_between)

    # land
    allcfs.land(targetHeight=0.06, duration=t_takeoff)
    timeHelper.sleep(t_takeoff + 1.0)

    print("sequence complete.")


def load_all_csvs(path):
    csvs = os.listdir(path)
    csvs = sorted(csvs, key=lambda s: int(os.path.splitext(s)[0])) # numerical order
    names, exts = zip(*[os.path.splitext(os.path.basename(f)) for f in csvs])
    assert all(e == ".csv" for e in exts)
    steps = len(names)
    assert set(names) == set([str(i) for i in range(1, steps + 1)])
    trajs = [uav_trajectory.Trajectory() for _ in range(steps)]
    for t, csv in zip(trajs, csvs):
        t.loadcsv(os.path.join(path, csv))
    return trajs


if __name__ == "__main__":
    main()
