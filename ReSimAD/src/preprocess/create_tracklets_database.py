import argparse
import copy
import glob
import json
import os
import pickle
import random

import tqdm
from waymo_open_dataset import dataset_pb2
import numpy as np
import tensorflow as tf
import math


def soft_mkdir(path):
    if os.path.exists(path) is False:
        os.mkdir(path)


class WaymoTracker(object):
    def __init__(self, dataset_dir: str, out_dir: str, kitti_size_adapt=False):
        self.extra_pedestrian = 15
        self.extra_cyclist = 10
        self.dataset_dir = dataset_dir
        self.out_dir = out_dir
        self.kitti_size_adapt = kitti_size_adapt
        if self.kitti_size_adapt:
            self.adapt_funcs = {'l': np.poly1d([0.10164198, 0.10128058, 1.27282786]),
                                'w': np.poly1d([0.09254018, 0.23633159, 0.74922371]),
                                'h': np.poly1d([0.69959114, -1.70475184, 2.39263071])}
        soft_mkdir(self.out_dir)
        self.focused_type = {1: 'car', 2: 'pedestrian', 4: 'cyclist'}
        self.history_label = {'pedestrian': [], 'cyclist': []}
        self.finish_list = []
        with open('./config/object_box_db.json', 'r') as f:
            self.bounding_box_db = json.load(f)

    def getsqr(self, x: float, y: float, z: float) -> float:
        return math.sqrt(math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2))

    def cal_euler_from_matrix(self, R):
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        if sy >= 1e-6:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([z, y, x])

    def get_foreground_object(self, frame):
        foreground_object_list = []
        for obj in frame.laser_labels:
            if obj.type not in self.focused_type:
                continue
            if obj.type == 1:  # vehicle
                location = [obj.box.center_x, -obj.box.center_y, obj.box.center_z - obj.box.height / 2]
            else:
                location = [obj.box.center_x, -obj.box.center_y, obj.box.center_z]
            box = [obj.box.length, obj.box.width, obj.box.height]
            if self.kitti_size_adapt:
                if 2.5 * 1.25 < box[0] < 6:
                    box[0] = self.adapt_funcs['l'](box[0])
                elif 1.5 < box[1] < 1.9:
                    box[1] = self.adapt_funcs['w'](box[1])
                elif 1.3 < box[2] < 2:
                    box[2] = self.adapt_funcs['h'](box[2])
            velocity = self.getsqr(obj.metadata.speed_x, obj.metadata.speed_y, 0)
            heading = -obj.box.heading
            angle = [math.degrees(heading), 0.0, 0.0]  # yaw,pitch,roll
            infolist = {'rot': angle, 'id': obj.id, 'box': box, 'loc': location,
                        'speed': velocity, 'type': self.focused_type[obj.type]}
            if obj.type == 2:
                total = random.randint(0, 34)
                left_roll = random.randint(0, total)
                right_roll = total + total - left_roll
                infolist.update({'left_roll': left_roll, 'right_roll': right_roll})
            if self.focused_type[obj.type] in self.history_label:
                fork_info = copy.deepcopy(infolist)
                fork_info['loc'] = np.dot(np.array(frame.pose.transform).reshape(4, 4)[0:3, 0:3],
                                          np.array(fork_info['loc'])) + np.array(frame.pose.transform).reshape(4, 4)[
                                                                        0:3,
                                                                        3]
                self.history_label[self.focused_type[obj.type]].append(fork_info)
            foreground_object_list.append(infolist)
        return foreground_object_list

    def x_to_world(self, pose):
        """
        The transformation matrix from x-coordinate system to carla world system

        Parameters
        ----------
        pose : list
            [x, y, z, roll, yaw, pitch]

        Returns
        -------
        matrix : np.ndarray
            The transformation matrix.
        """
        x, y, z, roll, yaw, pitch = pose[:]

        # used for rotation matrix
        c_y = np.cos(np.radians(yaw))
        s_y = np.sin(np.radians(yaw))
        c_r = np.cos(np.radians(roll))
        s_r = np.sin(np.radians(roll))
        c_p = np.cos(np.radians(pitch))
        s_p = np.sin(np.radians(pitch))

        matrix = np.identity(4)
        # translation matrix
        matrix[0, 3] = x
        matrix[1, 3] = y
        matrix[2, 3] = z

        # rotation matrix
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r

        return matrix

    def gen_extra_object(self, frames_tracklets, frames_pose):
        def isvalid(label, ego, foreground, extra, threshold):
            label_thre = np.sqrt(np.sum(np.square(label['box']))) / 2
            if np.abs(np.array(label['loc'])[0:2] - np.array([0, 0])).sum() < 10:
                return False
            for obj in foreground:
                if np.abs(np.array(label['loc'])[0:2] - np.array(obj['loc'][0:2])).sum() < label_thre + np.sqrt(
                        np.sum(np.square(obj['box']))) + threshold:
                    return False
            for obj in extra:
                if np.abs(np.array(label['loc'])[0:2] - np.array(obj['loc'][0:2])).sum() < label_thre + np.sqrt(
                        np.sum(np.square(obj['box']))) + threshold:
                    return False
            return True

        extra_list = []
        extra_cyclist_num = random.randint(self.extra_cyclist - 5, self.extra_cyclist)
        extra_pedestrian_num = random.randint(self.extra_pedestrian - 5, self.extra_pedestrian)
        for frame_idx, frame_info in enumerate(frames_tracklets):
            extra = []
            rotation = 0
            cur_frame_pose = frames_pose[frame_idx]
            cur_frame_pose_inv = np.linalg.inv(cur_frame_pose)
            total_pedestrian = len(self.history_label['pedestrian'])
            total_cyclist = len(self.history_label['cyclist'])
            if len(self.history_label['pedestrian']):
                for idx in range(extra_pedestrian_num):
                    ped = random.choice(self.history_label['pedestrian'])
                    ped = copy.deepcopy(ped)
                    ped['loc'] = np.dot(cur_frame_pose_inv[0:3, 0:3],
                                        np.array(ped['loc'])) + cur_frame_pose_inv[0:3, 3]
                    while rotation < min(100, total_pedestrian * 2) and \
                            isvalid(ped, frame_info['ego'], frame_info['foreground'], extra, 1) is False:
                        ped = random.choice(self.history_label['pedestrian'])
                        ped = copy.deepcopy(ped)
                        ped['loc'] = np.dot(cur_frame_pose_inv[0:3, 0:3],
                                            np.array(ped['loc'])) + cur_frame_pose_inv[0:3, 3]
                        rotation += 1
                    if isvalid(ped, frame_info['ego'], frame_info['foreground'], extra, 5):
                        extra.append(ped)
            rotation = 0
            if len(self.history_label['cyclist']):
                for idx in range(extra_cyclist_num):
                    cyc = random.choice(self.history_label['cyclist'])
                    cyc = copy.deepcopy(cyc)
                    cyc['loc'] = np.dot(cur_frame_pose_inv[0:3, 0:3],
                                        np.array(cyc['loc'])) + cur_frame_pose_inv[0:3, 3]
                    while rotation < min(100, total_cyclist * 2) and \
                            isvalid(cyc, frame_info['ego'], frame_info['foreground'], extra, 1) is False:
                        cyc = random.choice(self.history_label['cyclist'])
                        cyc = copy.deepcopy(cyc)
                        cyc['loc'] = np.dot(cur_frame_pose_inv[0:3, 0:3],
                                            np.array(cyc['loc'])) + cur_frame_pose_inv[0:3, 3]
                        rotation += 1
                    if isvalid(cyc, frame_info['ego'], frame_info['foreground'], extra, 5):
                        extra.append(cyc)
                extra_list.append(extra)
        return extra_list

    def get_ego_speed(self, frame):
        velocity = frame.images[0].velocity
        return self.getsqr(velocity.v_x, velocity.v_y, velocity.v_z)

    def getpose(self, frame):
        pose = np.array(frame.pose.transform).reshape(4, 4)
        angle = self.cal_euler_from_matrix(pose[:3, :3])
        loc = pose[:3, 3]
        angle = np.degrees(angle)
        return angle, loc, pose

    def match_bp_from_box(self, obj_type, box=None):
        bp_lib = self.bounding_box_db[obj_type]
        matched_bp_name = random.choice(list(bp_lib.keys()))
        if box is None:
            return matched_bp_name
        matched_dis = 1e10
        for bp, extent in bp_lib.items():
            dis = np.sqrt(np.square(np.subtract(box, extent)).sum())
            if dis < matched_dis:
                matched_dis = dis
                matched_bp_name = bp
        return matched_bp_name

    def convert_one(self, seq_path):
        dataset = tf.data.TFRecordDataset(seq_path, compression_type='')
        # dataset_name = dataset.context.name
        dataset_name = os.path.split(seq_path)[-1].split('.')[0].split('_with_camera_labels')[0]
        print(dataset_name)
        frames_tracklets = []
        self.history_label = {'pedestrian': [], 'cyclist': []}
        unique_objects = {}
        frames = []
        for frame_idx, data in tqdm.tqdm(enumerate(dataset)):
            frame = dataset_pb2.Frame()
            frame.ParseFromString(bytearray(data.numpy()))
            frames.append(np.array(frame.pose.transform).reshape(4, 4))
            ego_speed = self.get_ego_speed(frame)
            ego_angle, ego_loc, _ = self.getpose(frame)
            ego_loc[1] = -ego_loc[1]
            ego_angle = -ego_angle
            ego_data = {'id': 'ego', 'loc': ego_loc, 'rot': ego_angle, 'speed': ego_speed}
            foreground_object_list = self.get_foreground_object(frame)
            for obj in foreground_object_list:
                unique_objects[obj['id']] = {'type': obj['type'], 'box': obj['box']}
                # obj.pop('box')
                # obj.pop('type')

            data = {'ego': ego_data, 'foreground': foreground_object_list}
            frames_tracklets.append(data)
        for idx, obj in unique_objects.items():
            obj['bp'] = self.match_bp_from_box(obj['type'], obj['box'])
        extra_list = self.gen_extra_object(frames_tracklets, frames)
        for frame_idx, l, in enumerate(extra_list):
            frames_tracklets[frame_idx]['foreground'].extend(l)
        unique_objects['ego'] = {'type': 'car', 'box': np.array([3.9, 1.6, 1.56]),
                                 'bp': self.match_bp_from_box('car', np.array([3.9, 1.6, 1.56]))}
        seq_data = {'frames': frames_tracklets, 'objects': unique_objects}
        with open(os.path.join(self.out_dir, f'{str(dataset_name)}.pkl'), 'wb') as f:
            pickle.dump(seq_data, f)

    def convert(self):
        soft_mkdir(self.out_dir)
        self.finish_list = []
        dataset_sequences = glob.glob(os.path.join(self.dataset_dir, '*.tfrecord'))
        for idx, dataset_path in enumerate(dataset_sequences):
            print(idx, end=' ')
            # if dataset_path.find('segment-12896629105712361308_4520_000_4540_000') == -1:
            #     continue
            self.convert_one(dataset_path)
            self.finish_list.append(dataset_path)
        print("Finish dataset:")
        print(self.finish_list)


parser = argparse.ArgumentParser(description='extract tracklet from dataset')
parser.add_argument(
    '--root-path',
    type=str,
    default='/home/PJLAB/caixinyu/Documents/Waymo-Sim/waymo_sequence', # '/media/PJLAB\\caixinyu/Elements SE/WaymoSim/extra_seg',
help = 'specify the root path of dataset')
parser.add_argument(
    '--out-dir',
    type=str,
    default='./config/vec_track',
    required=False,
    help='name of info pkl')
args = parser.parse_args()

if __name__ == '__main__':
    random.seed = 2
    tracker = WaymoTracker(dataset_dir=args.root_path, out_dir=args.out_dir, kitti_size_adapt=False)
    tracker.convert()
