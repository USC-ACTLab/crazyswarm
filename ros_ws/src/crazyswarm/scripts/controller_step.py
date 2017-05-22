#!/usr/bin/env python

from __future__ import print_function

import time
from math import *
import numpy as np
import matplotlib.pyplot as plt

from crazyflie import *
from trajectory import *
import joystick

def main():
    server = CrazyflieServer()
    cf = server.crazyflies[0]
    Z = 1.0
    cf.takeoff(Z, Z + 1.0)
    time.sleep(Z + 4.0)

    p0 = cf.initialPosition + np.array([0, 0, Z])
    p1 = p0 + np.array([0, -0.3, 0])

    cf.hover(p0, 0, 1.0)
    time.sleep(8.0)

    ts = []
    ys = []

    t0 = time.clock()
    cf.hover(p1, 0, 1.0)
    while True:
        y = cf.position()[1]
        t = time.clock()
        if (t - t0) > 5.0:
            break
        ts.append(t)
        ys.append(y)

    cf.hover(p0, 0, 1.0)
    time.sleep(2.0)
    cf.land(0.06, Z + 1.0)

    plt.plot(ts, ys)
    plt.show()


if __name__ == "__main__":
    main()
