# -*-coding: utf-8 -*-
# vispy: gallery 30
# -----------------------------------------------------------------------------
# Copyright (c) 2015, Vispy Development Team. All Rights Reserved.
# Distributed under the (new) BSD License. See LICENSE.txt for more info.
# -----------------------------------------------------------------------------
"""
Simple use of SceneCanvas to display a cube with an arcball camera.
"""
import sys

from vispy import scene, app, io
from vispy.color import Color
from vispy.visuals import transforms
import time

canvas = scene.SceneCanvas(keys='interactive', size=(800, 600), show=True)

# Set up a viewbox to display the cube with interactive arcball
view = canvas.central_widget.add_view()
view.bgcolor = '#efefef'
view.camera = 'turntable'
view.padding = 100

color = Color("#3f51b5")

# cube = scene.visuals.Cube(size=1, color=color, edge_color="black",
#                           parent=view.scene)


verts, faces, normals, nothin = io.read_mesh("crazyflie2.obj.gz")

cfs = []
# cftransforms = []
for i in range(0, 10):
    mesh = scene.visuals.Mesh(vertices=verts, shading='smooth', faces=faces, parent=view.scene)
    mesh.transform = transforms.MatrixTransform()
    cfs.append(mesh)

# cube_transform = transforms.MatrixTransform()
# mesh.transform = cube_transform
theta = 0
lastTime = time.time()

def update(ev):
    global cfs, theta, lastTime
    x = 0
    for cf in cfs:
        cf.transform.reset()
        cf.transform.rotate(theta, (0, 0, 1))
        cf.transform.scale((0.01, 0.01, 0.01))
        cf.transform.translate((x, 0, 0))
        x += 1.0
    # cube_transform.reset()
    # cube_transform.rotate(theta, (0, 0, 1))
    theta += 1.0
    now = time.time()
    dt = now - lastTime
    print(1 / dt)
    lastTime = now

timer = app.Timer()
timer.connect(update)
timer.start(0)

if __name__ == '__main__' and sys.flags.interactive == 0:
    canvas.app.run()
