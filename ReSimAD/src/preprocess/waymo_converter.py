# Copyright (c) OpenMMLab. All rights reserved.
r"""Adapted from `Waymo to KITTI converter
    <https://github.com/caizhongang/waymo_kitti_converter>`_.
"""
import json
import math
import os.path
import pickle
import tqdm
from create_tracklets_database import WaymoTracker

try:
    from waymo_open_dataset import dataset_pb2
except ImportError:
    raise ImportError('Please run "pip install waymo-open-dataset-tf-2-5-0" '
                      '>1.4.5 to install the official devkit first.')

from glob import glob
from os.path import join

import numpy as np
import tensorflow as tf

CONVERT_ALL = 0
CONVERT_LIDAR = 1
CONVERT_OTHER = 2


class Waymo2KITTI(object):
    """Waymo to KITTI converter.

    This class serves as the converter to change the waymo raw data to KITTI
    format.

    Args:
        load_dir (str): Directory to load waymo raw data.
        point_cloud_root_dir (str): Directory to load lidar data.
        save_dir (str): Directory to save data in KITTI format.
    """

    def __init__(self,
                 load_dir,
                 point_cloud_root_dir,
                 save_dir,
                 prefix,
                 workers=16,
                 mode=CONVERT_ALL,
                 ):

        self.selected_waymo_classes = ['VEHICLE', 'PEDESTRIAN', 'CYCLIST']
        self.selected_waymo_locations = None
        # turn on eager execution for older tensorflow versions
        if int(tf.__version__.split('.')[0]) < 2:
            tf.enable_eager_execution()

        # keep the order defined by the official protocol
        self.cam_list = [
            '_FRONT',
            '_FRONT_LEFT',
            '_FRONT_RIGHT',
            '_SIDE_LEFT',
            '_SIDE_RIGHT',
        ]
        self.lidar_list = ['TOP', 'FRONT', 'SIDE_LEFT', 'SIDE_RIGHT', 'REAR']
        self.type_list = [
            'UNKNOWN', 'VEHICLE', 'PEDESTRIAN', 'SIGN', 'CYCLIST'
        ]
        self.waymo_to_kitti_class_map = {
            'UNKNOWN': 'DontCare',
            'PEDESTRIAN': 'Pedestrian',
            'VEHICLE': 'Car',
            'CYCLIST': 'Cyclist',
            'SIGN': 'Sign'  # not in kitti
        }
        self.load_dir = load_dir
        self.save_dir = save_dir
        self.point_cloud_root_dir = point_cloud_root_dir
        self.prefix = prefix
        self.workers = int(workers)
        self.vec_track_dir = './config/vec_track'
        if not os.path.isdir(self.save_dir):
            os.makedirs(self.save_dir)
        self.tfrecord_pathnames = sorted(
            glob(join(self.load_dir, '*.tfrecord')))
        self.sequence_index = {}
        self.convert_lidar = True
        self.convert_sequence = True
        if CONVERT_LIDAR == mode:
            self.convert_sequence = False
        elif CONVERT_OTHER == mode:
            self.convert_lidar = False
        with open('./config/sequence_index.txt', 'r') as f:
            # self.sequence_index = json.load(f)
            for line in f:
                line = line.split(' ')
                index = int(line[1])
                name = line[0]
                self.sequence_index[name] = index
        with open('./config/object_box_db.json', 'r') as f:
            self.bounding_box_db = json.load(f)
        self.label_carla_dir = f'{self.save_dir}/label_2'
        # self.label_adaptive_save_dir = f'{self.save_dir}/label_2'
        self.image_save_dir = f'{self.save_dir}/image_2'
        self.calib_save_dir = f'{self.save_dir}/calib'
        self.point_cloud_save_dir = f'{self.save_dir}/velodyne'
        self.create_folder()

    def convert(self):
        """Convert action."""
        print('Start converting ...')
        for i in range(len(self)):
            self.convert_one(i)
        print('\nFinished ...')

    def convert_one(self, file_idx):
        """Convert action for single file.

        Args:
            file_idx (int): Index of the file to be converted.
        """
        pathname = self.tfrecord_pathnames[file_idx]
        sequence_name = os.path.split(pathname)[-1].split('.')[0].split('_with_camera_label')[0]
        lidar_dir = f'{self.point_cloud_root_dir}/' + f'{sequence_name}' + '/npy'
        print('converting {}/{} '.format(file_idx, len(self)) + sequence_name)
        file_idx = self.sequence_index[sequence_name]
        if not os.path.isdir(lidar_dir):
            return
        with open(os.path.join(self.vec_track_dir, sequence_name + '.pkl'), 'rb') as f:
            track_db = pickle.load(f)

        dataset = tf.data.TFRecordDataset(pathname, compression_type='')

        for frame_idx, data in tqdm.tqdm(enumerate(dataset)):
            if self.convert_lidar:
                self.save_lidar(lidar_dir, file_idx, frame_idx)
            if self.convert_sequence:
                frame = dataset_pb2.Frame()
                frame.ParseFromString(bytearray(data.numpy()))
                if (self.selected_waymo_locations is not None
                        and frame.context.stats.location
                        not in self.selected_waymo_locations):
                    continue
                self.save_image(frame, file_idx, frame_idx)
                self.save_calib(frame, file_idx, frame_idx)
                self.save_label(frame, file_idx, frame_idx, track_db=track_db)

    def __len__(self):
        """Length of the filename list."""
        return len(self.tfrecord_pathnames)

    def save_lidar(self, lidar_dir, file_idx, frame_idx):
        lidar_path = f'{lidar_dir}/{(str(frame_idx).zfill(4))}.npy'
        velodyne_path = f'{self.point_cloud_save_dir}/' + \
                        f'{self.prefix}{str(file_idx).zfill(3)}' + \
                        f'{str(frame_idx).zfill(3)}.bin'
        pcd = np.load(lidar_path)
        pcd[..., 1] = -pcd[..., 1]
        pcd.tofile(velodyne_path)

    def save_image(self, frame, file_idx, frame_idx):
        """Parse and save the images in jpg format.

        Args:
            frame (:obj:`Frame`): Open dataset frame proto.
            file_idx (int): Current file index.
            frame_idx (int): Current frame index.
        """
        # for img in frame.images:
        img = frame.images[0]
        img_path = f'{self.image_save_dir}/' + \
                   f'{self.prefix}{str(file_idx).zfill(3)}' + \
                   f'{str(frame_idx).zfill(3)}.jpg'
        with open(img_path, 'wb') as fp:
            fp.write(img.image)

    def save_calib(self, frame, file_idx, frame_idx):
        """Parse and save the calibration data.

        Args:
            frame (:obj:`Frame`): Open dataset frame proto.
            file_idx (int): Current file index.
            frame_idx (int): Current frame index.
        """
        # waymo front camera to kitti reference camera
        T_front_cam_to_ref = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0],
                                       [1.0, 0.0, 0.0]])
        camera_calibs = []
        R0_rect = [f'{i:e}' for i in np.eye(3).flatten()]
        Tr_velo_to_cams = []
        calib_context = ''
        for camera in frame.context.camera_calibrations:
            # extrinsic parameters
            T_cam_to_vehicle = np.array(camera.extrinsic.transform).reshape(
                4, 4)
            T_vehicle_to_cam = np.linalg.inv(T_cam_to_vehicle)
            Tr_velo_to_cam = \
                self.cart_to_homo(T_front_cam_to_ref) @ T_vehicle_to_cam
            Tr_velo_to_cam[2][3] += 1.95  # for kitti velodyne lidar located at z=1.73 reltative to ground
            # Tr_velo_to_cam = self.cart_to_homo(T_front_cam_to_ref)
            if camera.name == 1:  # FRONT = 1, see dataset.proto for details
                self.T_velo_to_front_cam = Tr_velo_to_cam.copy()
            Tr_velo_to_cam = Tr_velo_to_cam[:3, :].reshape((12,))
            Tr_velo_to_cams.append([f'{i:e}' for i in Tr_velo_to_cam])
            # intrinsic parameters
            camera_calib = np.zeros((3, 4))
            camera_calib[0, 0] = camera.intrinsic[0]
            camera_calib[1, 1] = camera.intrinsic[1]
            camera_calib[0, 2] = camera.intrinsic[2]
            camera_calib[1, 2] = camera.intrinsic[3]
            camera_calib[2, 2] = 1
            camera_calib = list(camera_calib.reshape(12))
            camera_calib = [f'{i:e}' for i in camera_calib]
            camera_calibs.append(camera_calib)
        for i in range(4):
            calib_context += 'P' + str(i) + ': ' + \
                             ' '.join(camera_calibs[i]) + '\n'
        calib_context += 'R0_rect' + ': ' + ' '.join(R0_rect) + '\n'
        calib_context += 'Tr_velo_to_cam' + ': ' + \
                         ' '.join(Tr_velo_to_cams[0]) + '\n'
        Tr_imu_to_velo = np.identity(4, dtype=np.float32)
        Tr_imu_to_velo = Tr_imu_to_velo[:3, :].reshape((12,))
        Tr_imu_to_velo = [f'{i:e}' for i in Tr_imu_to_velo]
        calib_context += 'Tr_imu_to_velo' + ': ' + \
                         ' '.join(Tr_imu_to_velo) + '\n'

        with open(
                f'{self.calib_save_dir}/{self.prefix}' +
                f'{str(file_idx).zfill(3)}{str(frame_idx).zfill(3)}.txt',
                'w+') as fp_calib:
            fp_calib.write(calib_context)
            fp_calib.close()

    def save_label(self, frame, file_idx, frame_idx, track_db=None):
        """Parse and save the label data in txt format.
        The relation between waymo and kitti coordinates is noteworthy:
        1. x, y, z correspond to l, w, h (waymo) -> l, h, w (kitti)
        2. x-y-z: front-left-up (waymo) -> right-down-front(kitti)
        3. bbox origin at volumetric center (waymo) -> bottom center (kitti)
        4. rotation: +x around y-axis (kitti) -> +x around z-axis (waymo)

        Args:
            frame (:obj:`Frame`): Open dataset frame proto.
            file_idx (int): Current file index.
            frame_idx (int): Current frame index.
            cam_sync (bool, optional): Whether to save the cam sync labels.
                Defaults to False.
        """
        label_all_path = f'{self.label_carla_dir}/{self.prefix}' + \
                         f'{str(file_idx).zfill(3)}{str(frame_idx).zfill(3)}.txt'
        fp_carla = open(label_all_path, 'w+')
        for obj in track_db['frames'][frame_idx]['foreground']:
            bounding_box = (0, 0, 0, 0)
            id = obj['id']
            my_type = track_db['objects'][id]['type']
            my_type = str(my_type).capitalize()
            box3d = track_db['objects'][id]['box']
            loc = obj['loc']
            # loc = [0, -10, 0]
            rot = obj['rot']
            box = np.array(self.bounding_box_db[str(my_type).lower()][track_db['objects'][id]['bp']])
            box += np.random.normal(0, box / 150, box.shape)
            length, width, height = box[0], box[1], box[2]
            x = loc[0] - 1.5 + 0.595
            y = -loc[1]
            z = loc[2] - 1.85  # FOR KITTI PCD
            if my_type == 'Cyclist':
                height = box3d[2]
                length = box3d[0]
                width = box3d[1]
                z -= height / 2
            elif my_type == 'Pedestrian':
                z -= height / 2
                for fore_obj in track_db['frames'][frame_idx]['foreground']:
                    if fore_obj['id'] == id:
                        length = height / 4 * (math.sin(math.radians(fore_obj['left_roll'])) +
                                               math.sin(math.radians(fore_obj['right_roll']))) + length
                        break
            # project bounding box to the virtual reference frame
            pt_ref = self.T_velo_to_front_cam @ \
                     np.array([x, y, z, 1]).reshape((4, 1))
            x, y, z, _ = pt_ref.flatten().tolist()

            rotation_y = np.radians(rot[0]) - np.pi / 2
            truncated = 0
            occluded = 0
            alpha = -10

            line_carla_box = my_type + \
                             ' {} {} {} {} {} {} {} {} {} {} {} {} {} {}\n'.format(
                                 round(truncated, 2), occluded, round(alpha, 2),
                                 round(bounding_box[0], 2), round(bounding_box[1], 2),
                                 round(bounding_box[2], 2), round(bounding_box[3], 2),
                                 round(height, 2), round(width, 2), round(length, 2),
                                 round(x, 2), round(y, 2), round(z, 2),
                                 round(rotation_y, 2))
            fp_carla.write(line_carla_box)
        fp_carla.close()

    def create_folder(self):
        """Create folder for data preprocessing."""
        dir_list1 = [
            self.label_carla_dir,
            # self.label_adaptive_save_dir,
            self.calib_save_dir,
            self.image_save_dir,
            self.point_cloud_save_dir
        ]
        for d in dir_list1:
            self.mkdir_or_exist(d)

    def cart_to_homo(self, mat):
        """Convert transformation matrix in Cartesian coordinates to
        homogeneous format.
        """
        ret = np.eye(4)
        if mat.shape == (3, 3):
            ret[:3, :3] = mat
        elif mat.shape == (3, 4):
            ret[:3, :] = mat
        else:
            raise ValueError(mat.shape)
        return ret

    def mkdir_or_exist(self, d):
        if not os.path.isdir(d):
            os.mkdir(d)


if __name__ == '__main__':
    converter = Waymo2KITTI('./waymo_sequence/',
                            './data/synthetic_lidar_data/'
                            './data/kitti_like/', '',
                            mode=CONVERT_ALL)

    converter.convert()