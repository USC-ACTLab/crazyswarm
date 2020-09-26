#!/usr/bin/env python

from __future__ import print_function

import argparse
import os
import os.path

import numpy as np
import csv

from pycrazyswarm import *
import uav_trajectory

import pycrazyswarm.cfsim.cffirmware as firm


def main():

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("path",
        type=str,
        help="directory containing numbered subdirectories for each robot," +
            "each of which contains numbered <n>.csv files for each formation change")
    swarm = Crazyswarm(parent_parser=parser)
    args, unknown = parser.parse_known_args()

    #
    # DATA LOADING
    #
    folder_path = os.path.split(args.path)[0]
    print("folder_path:", folder_path)

    # ...capability matrices...
    with open(os.path.join(folder_path, "Capability_matrices.csv")) as f:
        read_data = csv.reader(f, delimiter=",")
        data = list(read_data)
        C_matrices = np.array(data).astype("int")

    # ...simulation parameters...
    with open(os.path.join(folder_path, "sim_parameter.txt")) as f:
        lines = [line.rstrip('\n') for line in f]
        n = int(lines[0])
        r = int(lines[1])

    # ...trajectory sequences...
    root = args.path
    robot_dirs = sorted(os.listdir(root), key=int)
    seqs = [load_all_csvs(os.path.join(root, d)) for d in robot_dirs]
    N = len(robot_dirs)
    steps = len(seqs[0])
    assert C_matrices.shape[0] == steps + 2, "capabilities / trajs mismatch"

    print("loading complete")

    #
    # DATA VALIDATION / PROCESSING
    #

    # transpose / reshape capabilities to (time, robot, capability)
    C_matrices = C_matrices.reshape((r, n, -1)).transpose([2, 1, 0])
    # lower brightness
    C_matrices = 0.6 * C_matrices

    # validate sequences w.r.t. each other
    assert all(len(seq) == steps for seq in seqs)
    for i in range(steps):
        agent_lens = [seq[i].duration for seq in seqs]
        assert all(agent_lens == agent_lens[0])
    step_lens = [t.duration for t in seqs[0]]

    print("validation complete")

    #
    # CRAZYSWARM INITIALIZATION
    #
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    crazyflies = allcfs.crazyflies

    # support trials on <N robots
    if len(crazyflies) < N:
        N = len(crazyflies)
        seqs = seqs[:N]
        C_matrices = C_matrices[:,:N,:]
    print("using", N, "crazyflies")

    # transposed copy - by timestep instead of robot
    seqs_t = zip(*seqs)

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

    # planners for takeoff and landing
    planners = [firm.planner() for cf in crazyflies]
    for p in planners:
        firm.plan_init(p)

    #
    # ASSORTED OTHER SETUP
    #

    # local helper fn to set colors
    def set_colors(i):
        for cf, color in zip(crazyflies, C_matrices[i]):
            cf.setLEDColor(*color)

    # timing parameters
    timescale = 1.0
    pause_between = 1.5
    takeoff_time = 3.0
    land_time = 4.0

    #
    # RUN DEMO
    #

    print("validation complete")

    # takeoff
    print("takeoff")
    z_init = traj_starts[0,2]

    for cf, p in zip(crazyflies, planners):
        p.lastKnownPosition = cf.position()
        vposition = firm.mkvec(*p.lastKnownPosition)
        firm.plan_takeoff(p, vposition, 0.0, z_init, takeoff_time, 0.0)

    poll_planners(crazyflies, timeHelper, planners, takeoff_time)
    end_pos = np.stack([cf.position() for cf in crazyflies])

    # set to full capability colors
    set_colors(0)

    # pause - all is well...
    hover(crazyflies, timeHelper, end_pos, pause_between)

    # set colors first capability loss
    set_colors(1)

    # pause - reacting to capability loss
    hover(crazyflies, timeHelper, end_pos, pause_between)

    # main loop!
    for step in range(steps):

        # move - new configuration after capability loss
        print("executing trajectory", step, "/", steps)
        poll_trajs(crazyflies, timeHelper, seqs_t[step], timescale)
        end_pos = np.stack([cf.position() for cf in crazyflies])

        # done with this step's trajs - hover for a few sec
        hover(crazyflies, timeHelper, end_pos, pause_between)

        # change the LEDs - another capability loss
        if step < steps - 1:
            set_colors(step + 2)

        # hover some more
        hover(crazyflies, timeHelper, end_pos, pause_between)

    # land
    print("landing")

    end_pos = np.stack([cf.position() for cf in crazyflies])
    for cf, p, pos in zip(crazyflies, planners, end_pos):
        vposition = firm.mkvec(*pos)
        firm.plan_land(p, vposition, 0.0, 0.06, land_time, 0.0)

    poll_planners(crazyflies, timeHelper, planners, land_time)

    # cut power
    print("sequence complete.")
    allcfs.emergency()

POLL_RATE = 100 # Hz

def poll_trajs(crazyflies, timeHelper, trajs, timescale):
    duration = trajs[0].duration
    start_time = timeHelper.time()
    while not timeHelper.isShutdown():
        t = (timeHelper.time() - start_time) / timescale
        if t > duration:
            break
        for cf, traj in zip(crazyflies, trajs):
            ev = traj.eval(t)
            cf.cmdFullState(
                ev.pos,
                ev.vel,
                ev.acc,
                ev.yaw,
                ev.omega)
        timeHelper.sleepForRate(POLL_RATE)


def poll_planners(crazyflies, timeHelper, planners, duration):
    start_time = timeHelper.time()
    while not timeHelper.isShutdown():
        t = timeHelper.time() - start_time
        if t > duration:
            break
        for cf, planner in zip(crazyflies, planners):
            ev = firm.plan_current_goal(planner, t)
            cf.cmdFullState(
                ev.pos,
                ev.vel,
                ev.acc,
                ev.yaw,
                ev.omega)
        timeHelper.sleepForRate(POLL_RATE)


def hover(crazyflies, timeHelper, positions, duration):
    start_time = timeHelper.time()
    zero = np.zeros(3)
    while not timeHelper.isShutdown():
        t = timeHelper.time() - start_time
        if t > duration:
            break
        for cf, pos in zip(crazyflies, positions):
            cf.cmdFullState(
                pos,
                zero,
                zero,
                0.0,
                zero)
        timeHelper.sleepForRate(POLL_RATE)


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
