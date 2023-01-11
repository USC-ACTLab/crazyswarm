#!/usr/bin/env python

from crazyflie_py import *


def main():
    Z = 1.0
    
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # disable LED (one by one)
    for cf in allcfs.crazyflies:
        cf.setParam("led.bitmask", 128)
        timeHelper.sleep(1.0)

    timeHelper.sleep(2.0)

    # enable LED (broadcast)
    allcfs.setParam("led.bitmask", 0)
    timeHelper.sleep(5.0)


if __name__ == "__main__":
    main()
