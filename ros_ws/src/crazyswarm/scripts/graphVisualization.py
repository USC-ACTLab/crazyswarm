#!/usr/bin/env python

"""Demonstrates the graph visualization feature of the 3D visualizer."""

import numpy as np

from pycrazyswarm import *

crazyflies_yaml = """
crazyflies:
- id: 1
  channel: 110
  initialPosition: [-1.0, 0.0, 0.0]
- id: 2
  channel: 120
  initialPosition: [-0.5, 0.0, 0.0]
- id: 3
  channel: 100
  initialPosition: [0.0, 0.0, 0.0]
- id: 4
  channel: 110
  initialPosition: [0.5, 0.0, 0.0]
- id: 5
  channel: 120
  initialPosition: [1.0, 0.0, 0.0]
"""

# Cycle between pentagon and star to show graph visualization.
t = np.linspace(0, 2 * np.pi, 6)[:-1]
positions = np.column_stack([np.cos(t), np.sin(t), np.ones_like(t)])
permutations = np.stack([np.arange(5), [0, 2, 4, 1, 3]])
graph_edges_pentagon = (
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 4),
    (4, 0),
)
graph_edges_star = (
    (0, 2),
    (0, 3),
    (1, 3),
    (1, 4),
    (2, 4),
)

if __name__ == "__main__":
    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml)
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    visualizer = timeHelper.visualizer
    visualizer.setGraph(graph_edges_star)

    cfs = allcfs.crazyflies

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(2.5)

    # First, cycle between pentagon and star by moving.
    n_cycles = 4
    for cycle in range(n_cycles):
        for i in range(5):
            index = permutations[cycle % 2][i]
            pos = positions[index]
            cfs[i].goTo(pos, 0, 3.0)

        timeHelper.sleep(6.0)

    # Then, cycle between pentagon and star by changing the graph.
    edges = (graph_edges_pentagon, graph_edges_star)
    for cycle in range(n_cycles):
        visualizer.setGraph(edges[cycle % 2])
        timeHelper.sleep(6.0)
