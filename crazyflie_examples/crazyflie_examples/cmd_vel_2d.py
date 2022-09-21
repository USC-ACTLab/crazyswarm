#!/usr/bin/env python

import numpy as np
from pathlib import Path
import time
from crazyflie_py import *

 
def executeCommand(timeHelper, cf, height, duration, rate=100,):


    start_time = timeHelper.time()
    while not timeHelper.isShutdown():
        t = timeHelper.time() - start_time
        if t > duration:
            break

        cf.cmdVelocity2D(0.0, 0.0, height, yawrate=0.2 )

        timeHelper.sleepForRate(rate)


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    rate = 30.0
    duration = 3.0
    Z = 0.5


    cf.takeoff(targetHeight=Z, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)

    executeCommand(timeHelper, cf, Z, duration, rate)

    time.sleep(0.5) #Necessary for Cflib backend since this is not implemented yet
    cf.notifySetpointsStop()
    cf.land(targetHeight=0.03, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)


if __name__ == "__main__":
    main()
