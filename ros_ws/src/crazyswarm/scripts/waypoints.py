#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

class Waypoint:
    def __init__(self, agent, x, y, z, arrival, duration):
        self.agent = agent
        self.x = x
        self.y = y
        self.z = z
        self.arrival = arrival
        self.duration = duration

    def __lt__(self, other):
        return self.arrival < other.arrival

    def __repr__(self):
        return "Ag {} at {} s. [{}, {}, {}]".format(self.agent, self.arrival, self.x, self.y, self.z)


if __name__ == "__main__":

    # load csv file
    data = np.loadtxt("waypoints.csv", skiprows=1, delimiter=',')

    # sort by agents
    data[data[:,0].argsort()]

    # convert to internal data structure
    waypoints = []
    lastAgent = None
    for row in data:
        if lastAgent is None or lastAgent != row[0]:
            lastTime = 0.0
        waypoints.append(Waypoint(
            int(row[0]),
            row[1],
            row[2],
            row[3],
            row[4],
            row[4] - lastTime))
        lastTime = row[4]
        lastAgent = int(row[0])

    # sort waypoints by arrival time
    waypoints.sort()

    # print waypoints
    print(waypoints)

    # execute waypoints
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(2.0)
    lastTime = 0.0
    for waypoint in waypoints:
        if waypoint.arrival == 0:
            pos = [waypoint.x, waypoint.y, waypoint.z]
            # print(waypoint.agent, pos, 2.0)
            cf = allcfs.crazyfliesById[waypoint.agent]
            cf.goTo(pos, 0, 2.0)
        elif waypoint.duration > 0:
            timeHelper.sleep(waypoint.arrival - lastTime)
            lastTime = waypoint.arrival
            pos = [waypoint.x, waypoint.y, waypoint.z]
            # print(waypoint.agent, pos, waypoint.duration)
            cf = allcfs.crazyfliesById[waypoint.agent]
            cf.goTo(pos, 0, waypoint.duration)

    # land
    allcfs.land(targetHeight=0.02, duration=2.0)
    timeHelper.sleep(2.0)
