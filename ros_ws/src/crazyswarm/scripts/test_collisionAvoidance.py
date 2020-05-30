import sys

import numpy as np

from pycrazyswarm import *
from pycrazyswarm.util import check_ellipsoid_collisions


crazyflies_yaml = """
crazyflies:
- id: 1
  channel: 100
  initialPosition: [-1.0, 0.0, 0.0]
- id: 2
  channel: 100
  initialPosition: [1.0, 0.0, 0.0]
"""

RADII = np.array([0.125, 0.125, 0.375])


def test_velocityMode_sidestepWorstCase(args=None):

    if args is None:
        args ="--sim --vis null --dt 0.05 --maxvel 1.0"

    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml, args=args)
    timeHelper = swarm.timeHelper
    a, b = swarm.allcfs.crazyflies

    a.enableCollisionAvoidance([b], RADII)
    b.enableCollisionAvoidance([a], RADII)

    a.cmdVelocityWorld([ 1.0, 0.0, 0.0], yawRate=0.0)
    b.cmdVelocityWorld([-1.0, 0.0, 0.0], yawRate=0.0)

    while timeHelper.time() < 10.0:
        positions = np.stack([a.position(), b.position()])
        collisions = check_ellipsoid_collisions(positions, RADII)
        assert not np.any(collisions)

        timeHelper.sleep(timeHelper.dt)
        if a.position()[0] > 1.0 and b.position()[0] < -1.0:
            return

    assert False


if __name__ == "__main__":
    test_velocityMode_sidestepWorstCase(sys.argv)
