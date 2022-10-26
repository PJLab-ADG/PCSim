import glob
import os
import sys
import time
from datetime import datetime
import math
from tkinter import HORIZONTAL
import weakref
import random
import pandas as  pd
import open3d as o3d
from matplotlib import cm
from numpy.core.arrayprint import set_string_function

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import Transform, Location, Rotation
import argparse
import logging
import random
import numpy as np

from plyfile import PlyData, PlyElement
import CoordConvertor

fused_lidar_data = None

# save point cloud(numpuy array to *.ply)
def create_ply(point_cloud, filename):
    points = [(point_cloud[i, 0], point_cloud[i, 1], point_cloud[i, 2], point_cloud[i, 3], point_cloud[i, 4], point_cloud[i, 5]) for i in range(point_cloud.shape[0])]
    vertex = np.array(points, dtype=[('x', 'f4'),('y', 'f4'),('z', 'f4'), ('CosAngle', 'f4'), ('ObjIdx', 'u4'), ('ObjTag', 'u4')])
    '''
    ply
    format ascii 1.0
    element vertex 92683
    property float32 x
    property float32 y
    property float32 z
    property float32 CosAngle
    property uint32 ObjIdx
    property uint32 ObjTag
    end_header
    '''
    e1 = PlyElement.describe(vertex, 'vertex')

    PlyData([e1], text=True).write(filename)

def lidar_callback(point_cloud, file_path, l2w_mat_target, l2w_mat, lidar_id):
    point_cloud.save_to_disk(os.path.join(file_path, '%06d.ply' % point_cloud.frame))
    # trans_mat = np.dot(l2w_mat_target.I, l2w_mat) # lidar_to_lidar1
    trans_mat = l2w_mat # lidar_to_world
    # if lidar_id != 1:
    #     CoordConvertor.pcd_coordinate_convertor(os.path.join(file_path, '%06d.ply' % point_cloud.frame), trans_mat)
    
    CoordConvertor.pcd_coordinate_convertor(os.path.join(file_path, '%06d.ply' % point_cloud.frame), trans_mat)



# def lidar_callback(point_cloud, file_path, l2w_mat_target, l2w_mat, lidar_id):
    # point_cloud.save_to_disk(os.path.join(file_path, '%06d.ply' % point_cloud.frame))

    # data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype([
    #     ('x', np.float32), ('y', np.float32), ('z', np.float32),
    #     ('CosAngle', np.float32), ('ObjIdx', np.uint32), ('ObjTag', np.uint32)])))
    # data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=[('x', 'f4'),('y', 'f4'),('z', 'f4'), ('CosAngle', 'f4'), ('ObjIdx', 'u4'), ('ObjTag', 'u4')]))
    # data = np.reshape(data, (int(data.shape[0] / 6), 6))

    # trans_mat = np.dot(l2w_mat_target.I, l2w_mat)
    # for line in data:
    #     if lidar_id != 1:
    #         loc = np.matrix([line[0], line[1], line[2], 1])
    #         loc_target = np.dot(trans_mat, loc)
    #         line[0] = loc_target[0]
    #         line[1] = loc_target[1]
    #         line[2] = loc_target[2]
    #     if fused_lidar_data == None:
    #         fused_lidar_data = line
    #     else:
    #         fused_lidar_data = np.concatenate((fused_lidar_data, line), axis=0)
    # if (lidar_id == 3):
    #     create_ply(fused_lidar_data, os.path.join(file_path+'fused', '%06d.ply' % point_cloud.frame))
    #     fused_lidar_data = None
        

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    # world =client.get_world()
    world = client.load_world("Town05")

    try:
        original_settings = world.get_settings()
        settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(8000)
        # traffic_manager.global_percentage_speed_difference(-133)
        traffic_manager.set_synchronous_mode(True)
        # delta = 0.01
        delta = 0.1
        settings.fixed_delta_seconds = delta
        settings.synchronous_mode = True
        world.apply_settings(settings)
        blueprint_library = world.get_blueprint_library()
        # --------------
        # Start recording
        # --------------
        """
        client.start_recorder('~/tutorial/recorder/recording01.log')
        """

        # --------------
        # Spawn ego vehicle
        # --------------

        ego_bp = world.get_blueprint_library().find('vehicle.lincoln.mkz_2017')
        ego_bp.set_attribute('role_name','ego')
        print('\nEgo role_name is set')
        # ego_color = random.choice(ego_bp.get_attribute('color').recommended_values)
        # ego_bp.set_attribute('color',ego_color)
        # print('\nEgo color is set')

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if 0 < number_of_spawn_points:
            random.shuffle(spawn_points)
            ego_transform = Transform(Location(x=-45,y=28,z=1.2), Rotation(pitch=0, yaw=-90, roll=0)) # 2
            # ego_transform = Transform(Location(x=-49,y=-10,z=1.2), Rotation(pitch=0, yaw=90, roll=0)) # 1
            # ego_transform = Transform(Location(x=115,y=250,z=2), Rotation(pitch=0, yaw=-90, roll=0)) # seeside

            # -31.636356353759766
            # 33.58928680419922
            # ego_vehicle = world.spawn_actor(ego_bp,ego_transform)
            print('\nEgo is spawned')
        else:
            logging.warning('Could not found any spawn points')

        # --------------
        # Add sensor to ego vehicle. 
        # --------------


        # lidar_location = carla.Location(x=-31.636356353759766,y=33.58928680419922,z=10)
        # lidar_rotation = carla.Rotation(0,0,0)
        # lidar_transform = carla.Transform(lidar_location,lidar_rotation)
        
        lidar_location1 = carla.Location(x=-180.0, y=-11.0, z=3.5) # 1
        lidar_location2 = carla.Location(x=-180.0, y=13.0, z=3.5) # 2
        lidar_location3 = carla.Location(x=-200.0, y=13.0, z=3.5) # 3
        lidar_location4 = carla.Location(x=-200.0, y=-11.0, z=3.5) # 4
        lidar_rotation1 = carla.Rotation(15,-45.0,0) # (pitch=0, yaw=-90, roll=0)
        lidar_rotation2 = carla.Rotation(15,45,0) # (pitch=0, yaw=-90, roll=0)
        lidar_rotation3 = carla.Rotation(-15,-45,0) # (pitch=0, yaw=-90, roll=0)
        lidar_rotation4 = carla.Rotation(-15,45,0) # (pitch=0, yaw=-90, roll=0)
        lidar_transform1 = carla.Transform(lidar_location1,lidar_rotation1)
        lidar_transform2 = carla.Transform(lidar_location2,lidar_rotation2)
        lidar_transform3 = carla.Transform(lidar_location3,lidar_rotation3)
        lidar_transform4 = carla.Transform(lidar_location4,lidar_rotation4)

        lidar_list = []

        pandar64_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
        pandar64_bp.set_attribute("lidar_type","mechanical") # choose lidar_type as mechanical
        pandar64_bp.set_attribute("name","pandar64") # choose lidar name as pandar64

        # ego_pandar64 = world.spawn_actor(pandar64_bp,lidar_transform)
        # # save_path = './pandar64_1/-30/' # 1
        # # save_path = './pandar64_2/15/' # 2
        # save_path = './pandar64_3/-30/' # 3
        # # save_path = './pandar64_4/' # 4
        # if not os.path.exists(save_path):
        #     os.makedirs(save_path)
        # ego_pandar64.listen(lambda data: data.save_to_disk(os.path.join(save_path, '%06d.ply' % data.frame)))
        # lidar_list.append(ego_pandar64)

        ego_pandar64_1 = world.spawn_actor(pandar64_bp,lidar_transform1)
        l2w_mat1 = np.matrix(lidar_transform1.get_matrix())
        np.savetxt('./lidar_to_world.txt', l2w_mat1, delimiter=' ')
        ego_pandar64_1.listen(lambda data: lidar_callback(data, './pandar64_1', l2w_mat1, l2w_mat1, 1))
        lidar_list.append(ego_pandar64_1)

        ego_pandar64_2 = world.spawn_actor(pandar64_bp,lidar_transform2)
        l2w_mat2 = np.matrix(lidar_transform2.get_matrix())
        ego_pandar64_2.listen(lambda data: lidar_callback(data, './pandar64_2', l2w_mat1, l2w_mat2, 2))
        lidar_list.append(ego_pandar64_2)

        ego_pandar64_3 = world.spawn_actor(pandar64_bp,lidar_transform3)
        l2w_mat3 = np.matrix(lidar_transform3.get_matrix())
        ego_pandar64_3.listen(lambda data: lidar_callback(data, './pandar64_3', l2w_mat1, l2w_mat3, 3))
        lidar_list.append(ego_pandar64_3)

        ego_pandar64_4 = world.spawn_actor(pandar64_bp,lidar_transform4)
        l2w_mat4 = np.matrix(lidar_transform4.get_matrix())
        ego_pandar64_4.listen(lambda data: lidar_callback(data, './pandar64_4', l2w_mat1, l2w_mat4, 4))
        lidar_list.append(ego_pandar64_4)

        # ego_pandar64_4 = world.spawn_actor(pandar64_bp,lidar_transform4)
        # lidar_list.append(ego_pandar64_4)


        # matrix = np.matrix(lidar_transform.get_matrix())
        # mat_path = os.path.join(save_path, 'lidar_to_world.txt')
        # np.savetxt(mat_path, matrix, delimiter=' ')


        # --------------
        # Place spectator on ego spawning
        # --------------
        spectator = world.get_spectator()
        # world_snapshot = world.wait_for_tick()
        spectator.set_transform(carla.Transform(lidar_transform1.location+carla.Location(z=50),carla.Rotation(pitch=-90,yaw=0)))

        # --------------
        # Enable autopilot for ego vehicle
        # --------------

        # ego_vehicle.set_autopilot(True)

        # --------------
        # Game loop. Prevents the script from finishing.
        # --------------
        frame=0
        dt0 = datetime.now()
        # ego_vehicle.apply_control(carla.VehicleControl(throttle=1, steer=0))
        i=0
        while True:
            world.tick()
            # while(process_time.total_seconds()<1):
            process_time = datetime.now() - dt0
            sys.stdout.write('\r' + 'FPS: ' + str(1.0 / process_time.total_seconds()))
            sys.stdout.flush()
            dt0 = datetime.now()

            time.sleep(0.01)
            dt0 = datetime.now()
            frame+=1
            # spectator = world.get_spectator()
            # # transform = ego_vehicle.get_transform()
            # # spectator.set_transform(carla.Transform(transform.location+carla.Location(z=50),carla.Rotation(pitch=-90,yaw=0)))
            # spectator.set_transform(carla.Transform(lidar_transform.location+carla.Location(z=50),carla.Rotation(pitch=0,yaw=0)))
 
    finally:
        # --------------
        # Stop recording and destroy actors
        # --------------
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)
        client.stop_recorder()
        # ego_vehicle.destroy()
        for lidar in lidar_list:
            lidar.destroy()
        

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone with tutorial_ego.')
