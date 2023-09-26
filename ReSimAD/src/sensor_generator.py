import glob
import io
import os
import queue
import sys
import weakref

import cv2
import numpy as np
from matplotlib import cm
import open3d as o3d
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from queue import Queue


def petrel_put_npy(client, path, array):
    print('saving ', path)
    with io.BytesIO() as f:
        np.save(f, array, allow_pickle=True)
        client.put(path, f.getvalue(), update_cache=True)


class Camera:
    pass


class LiDAR:
    def __init__(self, world, sequence, vis_o3d=False, save_local=False, save_petrel=False):
        self.vis_o3d = vis_o3d
        self.save_petrel = save_petrel
        self.save_local = save_local
        self.data_queue = Queue(maxsize=32)
        self.lidar_bp = None
        self.lidar = None
        self.data = None
        self.blueprint_library = world.get_blueprint_library()
        self.world = world
        self.full_init()
        if self.vis_o3d is True:
            self.vis_window = None
            self.pointcloud = None
            self.init_vis()
        if self.save_petrel is True:
            from petrel_client.client import Client
            self.petrel_client = Client('/home/PJLAB/caixinyu/.petreloss.conf')
            self.petrel_save_dir = os.path.join('s3://feiben/Reconstruction/waymo2kitti_sim/', sequence)
        if self.save_local is True:
            self.local_save_dir = os.path.join('./data/', sequence, 'npy')
            if not os.path.isdir(self.local_save_dir):
                os.makedirs(self.local_save_dir)

    def full_init(self):
        self.lidar_bp = self.generate_lidar_bp(self.blueprint_library,
                                               {'lidar_type': 'Surround', 'lidar_name': 'waymo_top'})

    def spawn_with_vehicle(self, vehicle):
        lidar_transform = carla.Transform(
            carla.Location(x=-0.595, z=1.85))  # )  # for kitti 1.73 # for waymo (x=0, z=1.95)
        lidar = self.world.spawn_actor(self.lidar_bp, lidar_transform, attach_to=vehicle)
        lidar.listen(lambda data: self.lidar_callback(data))
        self.lidar = lidar
        return self

    def init_vis(self):
        vis = o3d.visualization.Visualizer()
        vis.create_window(
            window_name='Carla Ego Lidar',
            width=960,
            height=540,
            left=480,
            top=270)
        vis.get_render_option().background_color = [0.05, 0.05, 0.05]
        vis.get_render_option().point_size = 1
        vis.get_render_option().show_coordinate_frame = True
        self.vis_window = vis
        self.pointcloud = o3d.geometry.PointCloud()
        self.pointcloud.points = o3d.utility.Vector3dVector(np.zeros((150000, 3)))
        self.pointcloud.colors = o3d.utility.Vector3dVector(np.ones((150000, 3)))
        # self.vis_window.add_geometry(self.pointcloud)

    def await_data(self, frame, timeout=8):
        timeout = time.time() + timeout * 60
        while time.time() < timeout:
            recv_frame, data, = self.data_queue.get(timeout=10)
            if frame == recv_frame:
                self.data = data[..., :4]
                return

    def lidar_callback(self, event):
        """Prepares a point cloud with intensity
        colors ready to be consumed by Open3D"""
        frame = event.frame
        data = np.copy(np.frombuffer(event.raw_data, dtype=np.dtype('f4')))
        data = np.reshape(data, (data.shape[0] // 5, 5))
        try:
            self.data_queue.put((frame, data), timeout=3)
        except queue.Full as e:
            print('LiDAR data queue is full...')
        print('received lidar frame ', data.shape)

    def generate_lidar_bp(self, blueprint_library, arg=None):
        """Generates a CARLA blueprint based on the script parameters"""
        if arg is None:
            arg = {}
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        # if arg.no_noise:
        lidar_bp.set_attribute('dropoff_general_rate', '0.0')
        lidar_bp.set_attribute('dropoff_intensity_limit', '0.0')
        lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
        # else:
        lidar_bp.set_attribute('noise_stddev', '0.0')
        lidar_bp.set_attribute('lidar_type', str(arg['lidar_type']))
        lidar_bp.set_attribute('name', str(arg['lidar_name']))
        return lidar_bp

    def tick(self, inter_frame, frame):
        self.await_data(frame)
        if self.vis_o3d:
            data = self.data[..., :4]
            points = data[:, :-1]
            points[:, :1] = -points[:, :1]
            # intensity = data[:, -1]
            if self.vis_o3d:
                # intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
                # int_color = np.c_[
                #     np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
                #     np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
                #     np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]
                int_color = np.ones_like(points)
                self.pointcloud.points = o3d.utility.Vector3dVector(points)
                self.pointcloud.colors = o3d.utility.Vector3dVector(int_color)
            if inter_frame == 0:
                self.vis_window.add_geometry(self.pointcloud)
            self.vis_window.update_geometry(self.pointcloud)
            self.vis_window.poll_events()
            self.vis_window.update_renderer()
            time.sleep(0.05)
        if self.save_petrel:
            save_path = os.path.join(self.petrel_save_dir, str(inter_frame).zfill(4) + '.npy')
            petrel_put_npy(self.petrel_client, save_path, self.data)
        if self.save_local:
            np.save(os.path.join(self.local_save_dir, str(inter_frame).zfill(4) + '.npy'), self.data)

    def destroy(self):
        if self.vis_o3d:
            self.vis_window.destroy_window()
        if self.lidar.is_alive:
            try:
                self.lidar.stop()
                self.lidar.destroy()
            except Exception as error:
                print(error)


class Camera:
    def __init__(self, world, relative_position='front', sequence='', camera_vis=True):
        self.camera_save = True
        self.data_queue = Queue()
        self.world = world
        self.camera_vis = camera_vis
        self.data = None
        self.save_path = os.path.join('./data/', sequence, 'camera')
        if not os.path.isdir(self.save_path):
            os.makedirs(self.save_path)
        self.image_width = 1920
        self.image_height = 1080
        self.relative_position = relative_position
        self.camera_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
        self.camera_blueprint.set_attribute('fov', '105')
        self.camera_blueprint.set_attribute("image_size_x", str(self.image_width))
        self.camera_blueprint.set_attribute("image_size_y", str(self.image_height))
        self.frame = 0

    def await_data(self, frame, timeout=8):
        timeout = time.time() + timeout * 60
        while time.time() < timeout:
            recv_frame, data, = self.data_queue.get(timeout=10)
            if frame == recv_frame:
                self.data = data
                return

    def spawn_with_vehicle(self, vehicle):
        camera_transform = self.spawn_point_estimation(self.relative_position)
        camera = self.world.spawn_actor(self.camera_blueprint, camera_transform, attach_to=vehicle)
        camera.listen(lambda data: self.camera_callback(data))
        self.camera = camera
        return self

    def spwan_at_transform(self):
        yaw, roll, pitch = 0, 0, -45
        carla_location = carla.Location(x=-15, y=35, z=20+28)
        carla_rotation = carla.Rotation(roll=roll, yaw=yaw, pitch=pitch)
        camera_transform = carla.Transform(carla_location, carla_rotation)
        camera = self.world.spawn_actor(self.camera_blueprint, camera_transform)
        camera.listen(lambda data: self.camera_callback(data))
        self.camera = camera
        return self

    def spawn_point_estimation(self, relative_position):
        carla_location = carla.Location(0, 0, 0)
        pitch = 0
        if relative_position == 'front':
            carla_location = carla.Location(x=carla_location.x + 2.5,
                                            y=carla_location.y,
                                            z=carla_location.z + 2.0)
            yaw = 0
        elif relative_position == 'bev':
            carla_location = carla.Location(x=carla_location.x + 20,
                                            y=carla_location.y - 20,
                                            z=carla_location.z + 25)
            yaw = 90
            roll = 0
            pitch = -40
        elif relative_position == 'right':
            carla_location = carla.Location(x=carla_location.x + 0.0,
                                            y=carla_location.y + 0.3,
                                            z=carla_location.z + 1.8)
            yaw = 100
        elif relative_position == 'left':
            carla_location = carla.Location(x=carla_location.x + 0.0,
                                            y=carla_location.y - 0.3,
                                            z=carla_location.z + 1.8)
            yaw = -100
        else:
            carla_location = carla.Location(x=carla_location.x - 2.0,
                                            y=carla_location.y,
                                            z=carla_location.z + 1.5)
            yaw = 180
        carla_rotation = carla.Rotation(roll=0, yaw=yaw, pitch=pitch)
        return carla.Transform(carla_location, carla_rotation)

    def camera_callback(self, event):
        mount_position = ['front', 'right', 'left', 'back', 'bev']
        idx = mount_position.index(self.relative_position)
        image = np.array(event.raw_data)
        image = image.reshape((self.image_height, self.image_width, 4))
        # we need to remove the alpha channel
        image = image[:, :, :3]
        self.data_queue.put((event.frame, image))

    def tick(self, inter_frame, frame):
        self.await_data(frame)
        names = ['front', 'right', 'left', 'back', 'bev']
        idx = names.index(self.relative_position)+
        rgb_image = np.array(self.data)
        if self.camera_save:
            cv2.imwrite(os.path.join(self.save_path, str(inter_frame).zfill(3) + '_camera%d' % idx + '.png'), rgb_image)
        if self.camera_vis:
            rgb_image = cv2.resize(rgb_image, (0, 0), fx=0.4, fy=0.4)
            cv2.imshow('%s camera' % self.relative_position, rgb_image)
            cv2.waitKey(1)

    def destroy(self):
        if self.camera.is_alive:
            try:
                self.camera.stop()
                self.camera.destroy()
            except Exception as error:
                print(error)


VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
LABEL_COLORS = np.array([
    (255, 255, 255),  # None
    (70, 70, 70),  # Building
    (100, 40, 40),  # Fences
    (55, 90, 80),  # Other
    (220, 20, 60),  # Pedestrian
    (153, 153, 153),  # Pole
    (157, 234, 50),  # RoadLines
    (128, 64, 128),  # Road
    (244, 35, 232),  # Sidewalk
    (107, 142, 35),  # Vegetation
    (0, 0, 142),  # Vehicle
    (102, 102, 156),  # Wall
    (220, 220, 0),  # TrafficSign
    (70, 130, 180),  # Sky
    (81, 0, 81),  # Ground
    (150, 100, 100),  # Bridge
    (230, 150, 140),  # RailTrack
    (180, 165, 180),  # GuardRail
    (250, 170, 30),  # TrafficLight
    (110, 190, 160),  # Static
    (170, 120, 50),  # Dynamic
    (45, 60, 150),  # Water
    (145, 170, 100),  # Terrain
]) / 255.0  # normalize each channel [0-1] since is what Open3D uses
