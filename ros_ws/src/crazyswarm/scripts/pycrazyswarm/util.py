"""Useful functions for both pycrazyswarm internals and user scripts."""

import numpy as np
import scipy as sp
import scipy.spatial


def check_ellipsoid_collisions(positions, radii):
    """Checks for collisions between a set of ellipsoids at given positions.

    Args:
        positions (array float[n, 3]): The ellipsoid centers.
        radii (array float[3]): The radii of the axis-aligned ellipsoids.

    Returns:
        colliding (array bool[n]): True at index i if the i'th ellipsoid
            intersects any of the other ellipsoids.
    """
    scaled = positions / radii[None, :]
    dists = sp.spatial.distance.pdist(scaled)
    dists = sp.spatial.distance.squareform(dists)
    # Do not consider 0 distance to self as a collision!
    n, _ = positions.shape
    dists[range(n), range(n)] = np.inf
    colliding = np.any(dists < 2.0, axis=1)
    return colliding


def yaml_with_positions(positions):
    return "crazyflies:" + "".join([
"""
- channel: 100
  id: {}
  initialPosition: {}""".format(i + 1, pos)
        for i, pos in enumerate(positions)
    ])
