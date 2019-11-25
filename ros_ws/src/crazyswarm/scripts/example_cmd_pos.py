from pycrazyswarm import *
import numpy as np

swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs

Z = 0
# takeoff
while Z < 1.0:
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
        cf.cmdPosition(pos)
    timeHelper.sleep(0.1)
    Z += 0.05

# land
while Z > 0.0:
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
        cf.cmdPosition(pos)
    timeHelper.sleep(0.1)
    Z -= 0.05

# turn-off motors
for cf in allcfs.crazyflies:
    cf.cmdStop()
