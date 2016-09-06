import os
import math
import numpy as np


class Output:
    def __init__(self):
        self.phase = 0
        self.data = dict()
        self.starttime = None

    def nextPhase(self):
        if self.phase > 0:
            for cfid, array in self.data.items():
                fileName = "cf{}_phase{}.csv".format(cfid, self.phase)
                np.savetxt(fileName, array, delimiter=',', newline='\n', header='t,x,y,z,roll,pitch,yaw')
        self.data = dict()
        self.phase += 1

    def update(self, t, crazyflies):
        for cf in crazyflies:
            x, y, z = cf.position()
            roll, pitch, yaw = cf.rpy()
            if cf.id not in self.data:
                self.data[cf.id] = np.empty((0, 7), float)
                self.starttime = t
            self.data[cf.id] = np.vstack([self.data[cf.id], np.array([t - self.starttime, x, y, z, roll, pitch, yaw])])
