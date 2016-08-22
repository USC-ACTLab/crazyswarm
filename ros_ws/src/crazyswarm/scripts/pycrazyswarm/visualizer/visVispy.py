import os
import math

from vispy import scene, app, io
from vispy.color import Color
from vispy.visuals import transforms
from vispy.scene.cameras import TurntableCamera


class VisVispy:
    def __init__(self):
        self.canvas = scene.SceneCanvas(keys='interactive', size=(800, 600), show=True)

        # Set up a viewbox to display the cube with interactive arcball
        self.view = self.canvas.central_widget.add_view()
        self.view.bgcolor = '#efefef'
        self.view.camera = TurntableCamera(fov=60.0) #'arcball'
        self.view.padding = 100

        # add a colored 3D axis for orientation
        axis = scene.visuals.XYZAxis(parent=self.view.scene)
        self.cfs = []

        ground = scene.visuals.Plane(1.0, 0.5, direction="+z", color=(0.5, 0.5, 1, 0.5), parent=self.view.scene)


    def update(self, t, crazyflies):
        if len(self.cfs) == 0:
            verts, faces, normals, nothin = io.read_mesh(os.path.join(os.path.dirname(__file__), "crazyflie2.obj.gz"))
            for i in range(0, len(crazyflies)):
                mesh = scene.visuals.Mesh(vertices=verts, shading='smooth', faces=faces, parent=self.view.scene)
                mesh.transform = transforms.MatrixTransform()
                self.cfs.append(mesh)

        for i in range(0, len(self.cfs)):
            x, y, z = crazyflies[i].position()
            self.cfs[i].transform.reset()
            self.cfs[i].transform.rotate(90, (1, 0, 0))
            # self.cfs[i].transform.rotate(math.pi / 2, (1, 0, 0))
            self.cfs[i].transform.scale((0.001, 0.001, 0.001))
            self.cfs[i].transform.translate((x, y, z))

        self.canvas.app.process_events()
