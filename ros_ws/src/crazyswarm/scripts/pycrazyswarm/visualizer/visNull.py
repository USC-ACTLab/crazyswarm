"""No-op visualizer for real hardware runs, so script can be oblivious."""

from collections import defaultdict

class VisNull:
    def __init__(self):
        # Always false for any key.
        self.keyState = defaultdict(bool)

    def setGraph(self, edges):
        pass

    def showEllipsoids(self, radii):
        pass

    def update(self, t, crazyflies):
        pass
