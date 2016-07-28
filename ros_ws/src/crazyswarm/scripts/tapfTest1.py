#!/usr/bin/env python

import json
import time

from crazyflie import *

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
    # read waypoints
    with open("continuousSchedule.json", 'r') as jsonfile:
        schedule = json.load(jsonfile)

    agentIdx = 0
    waypoints = []
    for agent in schedule["agents"]:
        lastTime = 0.0
        for p in agent["path"]:
            waypoints.append(Waypoint(
                agentIdx,
                float(p["x"]),
                float(p["y"]),
                float(p["z"]),
                float(p["arrival"]),
                float(p["arrival"]) - lastTime))
            lastTime = float(p["arrival"])
        agentIdx += 1

    # sort waypoints by arrival time
    waypoints.sort()

    print(waypoints)

    # execute waypoints
    allcfs = CrazyflieServer()

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    time.sleep(2.0)
    lastTime = 0.0
    for waypoint in waypoints:
        if waypoint.arrival == 0:
            pos = [waypoint.x, waypoint.y, waypoint.z]
            # print(waypoint.agent, pos, 2.0)
            cf = allcfs.crazyflies[waypoint.agent]
            cf.hover(pos, 0, 2.0)
        elif waypoint.duration > 0:
            time.sleep(waypoint.arrival - lastTime)
            lastTime = waypoint.arrival
            pos = [waypoint.x, waypoint.y, waypoint.z]
            # print(waypoint.agent, pos, waypoint.duration)
            cf = allcfs.crazyflies[waypoint.agent]
            cf.hover(pos, 0, waypoint.duration)


    # allcfs.land(targetHeight=0.06, duration=2.0)
    # time.sleep(3.0)
