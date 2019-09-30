import os
import math
import numpy as np


class Output:
    def __init__(self):
        self.data = dict()
        self.starttime = None

    def update(self, t, crazyflies):
        for cf in crazyflies:
            x, y, z = cf.position()
            roll, pitch, yaw = cf.rpy()
            if cf.id not in self.data:
                self.data[cf.id] = np.empty((0, 7), float)
                self.starttime = t
            self.data[cf.id] = np.vstack([self.data[cf.id], np.array([t - self.starttime, x, y, z, roll, pitch, yaw])])
