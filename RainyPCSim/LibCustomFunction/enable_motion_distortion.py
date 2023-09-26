from doctest import FAIL_FAST
import numpy as np
from plyfile import PlyData, PlyElement
import os

class LidarMotonDistortion:

    def __init__(self, file_path, delay_time):
        self.delay_time = delay_time
        self.point_cloud_list = []
        self.is_frame_enough = False
        self.file_path = file_path

        if not os.path.exists(file_path):
            os.makedirs(file_path)


    def enable_motion_distortion(self,point_cloud, enable = False):
        # point_cloud raw data
        # delay_frame = sim_frequency / lidar_frequency

        ply_filename = os.path.join(self.file_path, '%06d.ply' % point_cloud.frame)

        if (enable == False) or (self.delay_time ==0):
            point_cloud.save_to_disk(ply_filename)
            return

        data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
        data = np.reshape(data, (int(data.shape[0] / 5), 5))

        # save point cloud per delay_time frame
        self.point_cloud_list.append(data[:, :4])
        if (self.delay_time != 0) and (point_cloud.frame % self.delay_time == 0):
            if self.is_frame_enough == False:
                self.is_frame_enough = True
                self.point_cloud_list.clear()
                return

            data_write = None
            data_flag = True

            for dt in self.point_cloud_list:
                if data_flag:
                    data_write = dt
                    data_flag = False
                else:
                    data_write = np.concatenate((data_write, dt), axis=0)
            # data_write[:, 1] = -data_write[:, 1] # translate y to negtive(to right-handed)
            self.create_ply(data_write, ply_filename)        
            self.point_cloud_list.clear()


    # save point cloud(numpuy array to *.ply)
    def create_ply(self, point_cloud, filename):
        points = [(point_cloud[i, 0], point_cloud[i, 1], point_cloud[i, 2], point_cloud[i, 3]) for i in range(point_cloud.shape[0])]
        vertex = np.array(points, dtype=[('x', 'f4'),('y', 'f4'),('z', 'f4'), ('I', 'f4')])

        e1 = PlyElement.describe(vertex, 'vertex')

        PlyData([e1], text=True).write(filename)
    

    delay_time = 10
    point_cloud_list = []
    file_path = None
    is_frame_enough = False