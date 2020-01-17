import os
import math
import numpy as np

from vispy import scene, app, io
from vispy.color import Color
from vispy.visuals import transforms
from vispy.scene.cameras import TurntableCamera


CF_MESH_PATH = os.path.join(os.path.dirname(__file__), "crazyflie2.obj.gz")


class VisVispy:
    def __init__(self):
        self.canvas = scene.SceneCanvas(
            keys='interactive', size=(1024, 768), show=True, config=dict(samples=4), resizable=True
        )

        self.plane_color = 0.25 * np.ones((1, 3))
        self.bg_color = 0.9 * np.ones((1, 3))
        self.line_color = 0.7 * np.ones((1, 3))

        # Set up a viewbox to display the cube with interactive arcball
        self.view = self.canvas.central_widget.add_view()
        self.view.bgcolor = self.bg_color
        self.view.camera = TurntableCamera(
            fov=30.0, elevation=30.0, azimuth=90.0, center=(0.0, 0.0, 1.25)
        )
        self.cam_state = self.view.camera.get_state()

        # add a colored 3D axis for orientation
        axis = scene.visuals.XYZAxis(parent=self.view.scene)
        self.cfs = []
        self.color_cache = []

        ground = scene.visuals.Plane(
            32.0, 32.0, direction="+z", color=self.plane_color, parent=self.view.scene
        )

        # Lazy-constructed vispy objects and data for connectivity graph gfx.
        self.graph_edges = None
        self.graph_lines = None
        self.graph = None

    def setGraph(self, edges):
        """Set edges of graph visualization - sequence of (i,j) tuples."""

        # Only allocate new memory if we need to.
        n_edges = len(edges)
        if self.graph_edges is None or n_edges != len(self.graph_edges):
            self.graph_lines = np.zeros((2 * n_edges, 3))
        self.graph_edges = edges

        # Lazily construct VisPy object for graph.
        if self.graph is None:
            self.graph = scene.visuals.Line(
                parent=self.view.scene,
                color=self.line_color,
                pos=self.graph_lines,
                connect="segments",
                method="gl",
                antialias=True,
            )

    def update(self, t, crazyflies):
        if len(self.cfs) == 0:
            verts, faces, normals, nothin = io.read_mesh(CF_MESH_PATH)
            for i, cf in enumerate(crazyflies):
                color = cf.ledRGB
                mesh = scene.visuals.Mesh(
                    parent=self.view.scene,
                    vertices=verts,
                    faces=faces,
                    color=color,
                    shading="smooth",
                )
                mesh.light_dir = (0.1, 0.1, 1.0)
                mesh.shininess = 0.01
                mesh.ambient_light_color = [0.5] * 3
                mesh.transform = transforms.MatrixTransform()
                self.cfs.append(mesh)
                self.color_cache.append(color)

        positions = np.stack(cf.position() for cf in crazyflies)

        for i in range(0, len(self.cfs)):
            x, y, z = crazyflies[i].position()
            roll, pitch, yaw = crazyflies[i].rpy()
            self.cfs[i].transform.reset()
            self.cfs[i].transform.rotate(-90, (1, 0, 0))
            self.cfs[i].transform.rotate(math.degrees(roll), (1, 0, 0))
            self.cfs[i].transform.rotate(math.degrees(pitch), (0, 1, 0))
            self.cfs[i].transform.rotate(math.degrees(yaw), (0, 0, 1))
            self.cfs[i].transform.scale((0.002, 0.002, -0.002))
            self.cfs[i].transform.translate(positions[i])
            # vispy does not do this check
            color = crazyflies[i].ledRGB
            if color != self.color_cache[i]:
                self.color_cache[i] = color
                self.cfs[i].color = color  # sets dirty flag

        # Update graph line segments to match new Crazyflie positions.
        if self.graph is not None:
            for k, (i, j) in enumerate(self.graph_edges):
                self.graph_lines[2 * k, :] = positions[i]
                self.graph_lines[2 * k + 1, :] = positions[j]
            self.graph.set_data(self.graph_lines)

        self.canvas.app.process_events()
