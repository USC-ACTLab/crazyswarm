import datetime
import os
from pathlib import Path

import bpy
import numpy as np
from rclpy.node import Node
import rowan as rw
import yaml

from ..sim_data_types import Action, State


# rotation vectors are axis-angle format in 'compact form', where
# theta = norm(rvec) and axis = rvec / theta
# they can be converted to a matrix using cv2. Rodrigues, see
# https://docs.opencv.org/4.7.0/d9/d0c/group__calib3d.html#ga61585db663d9da06b68e70cfbf6a1eac
def opencv2quat(rvec):
    angle = np.linalg.norm(rvec)
    if angle == 0:
        q = np.array([1, 0, 0, 0])
    else:
        axis = rvec.flatten() / angle
        q = rw.from_axis_angle(axis, angle)
    return q


class Visualization:

    def __init__(self, node: Node, params: dict, names: list[str], states: list[State]):
        self.node = node

        # blender
        # load environment
        world = bpy.context.scene.world
        world.use_nodes = True
        self.env = world.node_tree.nodes.new('ShaderNodeTexEnvironment')
        if params['cycle_bg']:
            bg_paths = []
            p = Path(__file__).resolve().parent / 'data/env'
            print(p)
            for subdir, dirs, files in os.walk(p):
                bg_paths.extend([os.path.join(subdir, file) for file in files])
            self.cycle_bg = True
            self.bg_idx = 0
            self.bg_imgs = [bpy.data.images.load(bgp) for bgp in bg_paths]
            print(self.bg_imgs)
            print(bg_paths)
            self.env.image = self.bg_imgs[self.bg_idx]
        else:
            self.cycle_bg = False
            self.env.image = bpy.data.images.load(
                Path(__file__).resolve().parent / 'data/env/env.jpg')
        node_tree = world.node_tree
        node_tree.links.new(
            self.env.outputs['Color'],
            node_tree.nodes['Background'].inputs['Color'])

        # import crazyflie object
        bpy.ops.import_scene.obj(
            filepath=f'{Path(__file__).resolve().parent}/data/model/cf.obj',
            axis_forward='Y', axis_up='Z')
        self.cf_default = bpy.data.objects['cf']
        # save scene
        self.scene = bpy.context.scene
        self.scene.render.resolution_x = 320
        self.scene.render.resolution_y = 320
        self.scene.render.pixel_aspect_x = 1.0
        self.scene.render.pixel_aspect_y = 1.0
        self.scene.render.image_settings.file_format = 'JPEG'
        self.scene.unit_settings.length_unit = 'METERS'
        self.scene.unit_settings.system = 'METRIC'
        self.scene.unit_settings.scale_length = 1.0
        # self.scene.render.threads = 2  # max CPU cores to use to render

        # remove default objects
        bpy.data.objects.remove(bpy.data.objects['Cube'])
        bpy.data.objects.remove(bpy.data.objects['Light'])
        # create lamp
        lamp_data = bpy.data.lights.new(name='Lamp', type='SUN')
        lamp_data.energy = 1.5
        lamp_data.angle = 0.19198621809482574  # 11 deg
        self.lamp = bpy.data.objects.new(name='Lamp', object_data=lamp_data)
        bpy.context.collection.objects.link(self.lamp)
        bpy.context.view_layer.objects.active = self.lamp
        # camera
        self.camera = bpy.data.objects['Camera']
        self.camera.data.lens = 0.7376461029052734
        self.camera.data.lens_unit = 'FOV'
        self.camera.data.sensor_fit = 'AUTO'
        self.camera.data.sensor_width = 1.4
        self.camera.data.sensor_height = 18
        self.camera.data.angle = 1.518436431884765  # 87 deg
        self.camera.data.clip_start = 1.1e-6
        # link camera to scene

        self.cf_default.hide_render = False
        # set rotation mode to quaternion
        self.cf_default.rotation_mode = 'QUATERNION'
        self.camera.rotation_mode = 'QUATERNION'
        self.lamp.rotation_mode = 'QUATERNION'

        base = 'simulation_results'
        self.path = base + '/' + datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S') + '/'
        os.makedirs(self.path, exist_ok=True)

        self.ts = []
        self.frame = 0
        self.fps = params['fps']  # frames per second

        self.names = names
        self.n = len(names)
        self.state_filenames = []
        self.cam_state_filenames = []

        # dictionary with (name, idx) pairs to later find corresponding cf
        self.names_idx_map = {}
        # init list
        self.cf_list = [self.cf_default]
        self.cf_cameras = params['cf_cameras'] if 'cf_cameras' in params else {}
        # matrix where rows are the constant transformation between camera and its cf
        # it should only be accessed if corresponding cf carries a camera
        self.Q_virt_cf_cam = np.zeros((self.n, 4))

        for idx, (name, state) in enumerate(zip(names, states)):
            self.names_idx_map[name] = idx
            if idx > 0:
                # cf copies
                cf_copy = self.cf_default.copy()
                bpy.context.collection.objects.link(cf_copy)
                self.cf_list.append(cf_copy)
            # set rotations
            self.cf_list[idx].rotation_quaternion = np.array(state.quat)
            # set positions
            self.cf_list[idx].location = np.array(state.pos)

        self.tvecs = np.zeros((self.n, 3))
        for name in names:
            os.mkdir(self.path + '/' + name + '/')  # create dir for every cf for saving images
            csf = f'{self.path}/{name}/{name}.csv'
            calibration_sf = f'{self.path}/{name}/calibration.yaml'
            # initialize <robot_name>/<robot_name>.csv
            self.state_filenames.append(csf)
            # self.cam_state_filenames.append(cam_sf)
            with open(csf, 'w') as file:
                file.write('image_name,timestamp,x,y,z,qw,qx,qy,qz\n')
            if name in self.cf_cameras:
                calibration = self.cf_cameras[name]['calibration']
                calibration['dist_coeff'] = np.zeros(5).tolist()
                calibration['camera_matrix'] = np.array([
                    [170.0, 0, 160.0],
                    [0, 170.0, 160.0],
                    [0, 0, 1]]).tolist()
                with open(calibration_sf, 'w') as file:
                    yaml.dump(calibration, file)
                rvec = np.array([1.2092, -1.2092, 1.2092])
                if 'rvec' in calibration:
                    rvec = np.array(calibration['rvec'])
                tvec = np.array(calibration['tvec']) if 'tvec' in calibration else np.zeros(3)
                self.tvecs[self.names_idx_map[name]] = tvec
                q_real_camera_to_robot = rw.inverse(opencv2quat(rvec))
                q_virtual_camera_to_real_camera = rw.from_euler(np.pi, 0, 0, 'xyz')
                self.Q_virt_cf_cam[self.names_idx_map[name]] = rw.multiply(
                    q_real_camera_to_robot,
                    q_virtual_camera_to_real_camera)

    def step(self, t, states: list[State], states_desired: list[State], actions: list[Action]):
        self.ts.append(t)
        # render and record `self.fps` frames per second
        if t - self.frame / self.fps >= 1 / self.fps:
            self.frame += 1
            # quaternion matrix
            Q = np.zeros((self.n, 4))
            # position matrix
            P = np.zeros((self.n, 3))

            # first put everything in place and record cfs's states
            for name, state in zip(self.names, states):
                idx = self.names_idx_map[name]
                Q[idx] = np.array(state.quat)
                P[idx] = np.array(state.pos)

                # set rotations
                self.cf_list[idx].rotation_quaternion = Q[idx]
                # set positions
                self.cf_list[idx].location = P[idx]
                # record states
                # image capturing scene from cf's pov or None
                image_name = None
                if name in self.cf_cameras:
                    image_name = f'{self.names[idx]}_{self.frame:05}.jpg'
                # record cf's state in world frame
                with open(self.state_filenames[idx], 'a') as file:
                    file.write(f'{image_name},{t},{P[idx,0]},{P[idx,1]},{P[idx,2]},{Q[idx,0]},{Q[idx,1]},{Q[idx,2]},{Q[idx,3]}\n')  # noqa E501

            # cycle background image if enabled
            if self.cycle_bg:
                self.bg_idx = (self.bg_idx + 1) % len(self.bg_imgs)
                self.env.image = self.bg_imgs[self.bg_idx]

            # render images from cfs' perspectives
            for name, state in zip(self.names, states):
                idx = self.names_idx_map[name]
                if name not in self.cf_cameras:
                    continue
                # rotation
                self.camera.rotation_quaternion = rw.multiply(Q[idx], self.Q_virt_cf_cam[idx])
                self.lamp.rotation_quaternion = self.camera.rotation_quaternion
                # positions
                p_cam = P[idx] + rw.to_matrix(Q[idx]) @ self.tvecs[idx]
                self.camera.location = p_cam
                self.lamp.location = p_cam
                # hide corresponding cf for rendering
                self.cf_list[idx].hide_render = True
                # Render image
                image_name = f'{name}_{self.frame:05}.jpg'  # image capturing scene from cf's pov
                self.scene.render.filepath = f'{self.path}/{name}/{image_name}'
                bpy.ops.render.render(write_still=True)
                # show again after rendering
                self.cf_list[idx].hide_render = False

    def shutdown(self):
        for idx in range(1, self.n):
            bpy.data.objects.remove(self.cf_list[idx])
