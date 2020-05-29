#!/usr/bin/env python

"""Demonstrate running a slow computation without blocking the main thread.

Computations that take more than a few milliseconds, such as multi-robot
planning algorithms, often cannot run in the main script thread:

1. In simulation, the 3d graphics will not update during a blocking call.
2. In a streaming setpoint mode such as cmdVelocityWorld, the hardware expects
   to receive setpoints several times per second, even if the setpoint is
   constant.

Therefore, we must run the computations in a separate thread or process while
the main thread (1) allows the visualizer to update, and/or (2) sends a
setpoint on the radio.

Due to the global interpreter lock (GIL) in CPython, it's better to use
processs instead of threads. We illustrate one method here, including passing
arguments to the long-running function and receiving outputs using a queue.
"""

from __future__ import print_function
import multiprocessing
import time

import numpy as np

from pycrazyswarm import *


crazyflies_yaml = """
crazyflies:
- channel: 100
  id: 1
  initialPosition: [1.0, 0.0, 0.0]
"""

Z = 1.0  # Altitude.


def slow(output_queue, seconds):
    time.sleep(seconds)
    queue.put("OK")


if __name__ == "__main__":

    # The amount of time the slow computation will take. Also demonstrates how
    # to pass args.
    computation_time = 4.0

    # The queue is used to get outputs back from the background process.
    queue = multiprocessing.Queue()

    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml)
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    # Demonstration using high-level commands: timeHelper.sleep loops for us.
    p = multiprocessing.Process(target=slow, args=(queue, computation_time))
    p.start()
    print("first process started.")
    cf.takeoff(targetHeight=Z, duration=2.0)
    timeHelper.sleep(3.0)
    p.join()
    print("result:", queue.get())

    # Demonstration using low-level setpoints.
    p = multiprocessing.Process(target=slow, args=(queue, computation_time))
    p.start()
    print("second process started")

    t0 = timeHelper.time()
    while p.is_alive():
        t = timeHelper.time() - t0
        x = np.cos(t)
        y = np.sin(t)
        cf.cmdPosition([x, y, Z], yaw=0.0)
        timeHelper.sleep(timeHelper.dt)

    print("result:", queue.get())
