#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

from __future__ import print_function

import glob
import os
import pickle
import sys

from walker_control import BaseWalkerPose

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

import argparse
import logging
import re
import numpy as np

"""
Import Waymo
"""
from waymo_open_dataset import dataset_pb2


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


def get_actor_blueprints(blueprint_library, filter):
    bps = blueprint_library.filter(filter)
    if len(bps) == 1:
        return bps[0]
    elif len(bps) < 1:
        return []
    return bps


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, waymo_sequence):
        self.inter_frame = 0
        self.world = carla_world
        self.blueprint_library = None
        self.track_db_path = './config/vec_track'
        self.waymo_sequence = str(waymo_sequence).strip('\n')
        self.mesh_bp_name = self.waymo_sequence.split('-')[0][0:3] + self.waymo_sequence.split('-')[1][0:7]
        self.waymo_sequence_path = os.path.join(self.track_db_path, self.waymo_sequence + '.pkl')
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            sys.exit(1)
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self.player = None
        self.sensors = []
        self.mesh_bp = None
        self.mesh = None

        self.track_db = None
        self.foreground_blueprint_library = None
        self.frames = None
        self.ego_blueprint = None
        self.base_transform = None
        self.waymo_base_location = None
        self.waymo_base_rotation = None
        self.ego2lidar_bias = 1.5

        self.spawned_foreground_object = []
        self.full_init()
        self.setup()

    def full_init(self):
        self.blueprint_library = self.world.get_blueprint_library()
        with open(self.waymo_sequence_path, 'rb') as f:
            self.track_db = pickle.load(f)
        self.foreground_blueprint_library = self.track_db['objects']
        for obj in self.foreground_blueprint_library.values():
            obj['bp'] = get_actor_blueprints(self.blueprint_library, obj['bp'])
        self.frames = self.track_db['frames']

        self.base_transform = tuple([0, 0, 20])
        self.bias_transform = tuple([0, 0, 0])
        self.waymo_base_location = tuple(self.frames[0]['ego']['loc'])
        self.waymo_base_rotation = tuple(self.frames[0]['ego']['rot'])
        self.pedestrian_controller = BaseWalkerPose()
        self.ego_blueprint = get_actor_blueprints(self.blueprint_library,
                                                  'vehicle.lincoln.mkz_2020')  # self.foreground_blueprint_library['ego']['bp']
        # setup mesh
        self.mesh_bp = get_actor_blueprints(self.blueprint_library, self.mesh_bp_name)

    def destroy_foreground_object(self):
        # destroy all actor except ego
        for actor in self.spawned_foreground_object:
            if actor.is_alive is True:
                try:
                    actor.destroy()
                except Exception as e:
                    print(e, 'destroy failed')

    def spawn_foreground_object(self, object_list):
        for object_label in object_list:
            angle = object_label['rot']
            speed = object_label['speed']
            location = object_label['loc']
            location[0] = location[0] - self.ego2lidar_bias
            bp = self.foreground_blueprint_library.get(object_label['id'])['bp']
            type = self.foreground_blueprint_library.get(object_label['id'])['type']
            if speed >= -1:
                spawn_transform = carla.Transform(
                    carla.Location(*tuple(location)),
                    carla.Rotation(roll=angle[-2], pitch=-angle[-1], yaw=angle[0]))
                spawned_object = self.world.try_spawn_actor(bp, spawn_transform, attach_to=self.player)
                if spawned_object is None:
                    print('spawn failed ...')
                else:
                    if type == 'pedestrian':
                        self.pedestrian_controller.control(spawned_object, left_roll=object_label['left_roll'],
                                                           right_roll=object_label['right_roll'])
                    spawned_object.set_simulate_physics(False)
                    self.spawned_foreground_object.append(spawned_object)

    def refresh_traffic_flow(self):
        frame = self.frames[self.inter_frame]
        self.destroy_foreground_object()
        ego_loc, ego_angle = frame['ego']['loc'], frame['ego']['rot']
        # ego transform update
        ego_rotation = carla.Rotation(roll=-ego_angle[2], pitch=ego_angle[1], yaw=ego_angle[0])
        ego_bias = ego_rotation.get_forward_vector() * self.ego2lidar_bias
        ego_bias = [ego_bias.x, ego_bias.y, ego_bias.z]
        new_ego_loc = np.array(ego_loc) - np.array(self.waymo_base_location)
        new_ego_loc = np.array(self.base_transform[0:3]) + new_ego_loc + np.array(self.bias_transform) + np.array(ego_bias)
        new_ego_ue_trans = carla.Transform(carla.Location(*tuple(new_ego_loc)), ego_rotation)
        self.player.set_transform(new_ego_ue_trans)
        self.spawn_foreground_object(frame['foreground'])

        return self.inter_frame < len(self.frames)

    def tick(self, frame):
        for sensor in self.sensors:
            sensor.tick(self.inter_frame, frame)
        self.inter_frame += 1
        print(self.player.get_location())

    def setup(self):
        # clean up
        self.destroy_all()

        mesh_transform = carla.Transform(carla.Location(*self.base_transform),
                                         carla.Rotation(0, 0, 0))
        self.mesh = self.world.try_spawn_actor(self.mesh_bp, mesh_transform)
        # setup blueprint.
        blueprint = self.ego_blueprint
        # blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # spawn ego car
        spawn_point_ego_car = carla.Transform(carla.Location(*self.base_transform),
                                              carla.Rotation(roll=-self.waymo_base_rotation[-1],
                                                             pitch=self.waymo_base_rotation[-2],
                                                             yaw=self.waymo_base_rotation[-3]))
        self.player = self.world.try_spawn_actor(blueprint, spawn_point_ego_car)
        if self.player is None:
            print('player is None')
            raise Exception('ego spawn failed')
        self.player.set_simulate_physics(False)

        # setup sensors
        from sensor_generator import LiDAR, Camera
        lidar = LiDAR(self.world, sequence=self.waymo_sequence, vis_o3d=False, save_local=True, save_petrel=False)
        front_camera = Camera(self.world, 'front', self.waymo_sequence, False)
        bev_camera = Camera(self.world, 'bev', self.waymo_sequence, False)
        self.sensors.append(lidar)
        self.sensors.append(front_camera)
        for sensor in self.sensors:
            sensor.spawn_with_vehicle(self.player)
        bev_camera.spwan_at_transform()
        self.sensors.append(bev_camera)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.player.get_world().set_weather(preset[0])

    def destroy_all(self):
        for sensor in self.sensors:
            if sensor is not None:
                sensor.destroy()
        self.destroy_foreground_object()
        if self.player is not None and self.player.is_alive:
            self.player.destroy()
        if self.mesh is not None and self.mesh.is_alive:
            self.mesh.destroy()


def replay_all(args):
    if not os.path.exists(args.waymo_sequence_txt):
        raise Exception('no such file' + 'waymo_sequence_txt')
    sequences = []
    with open(args.waymo_sequence_txt, 'r') as f:
        # self.sequence_index = json.load(f)
        for line in f:
            sequences.append(line.split(' ')[0])
    world = None
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    sim_world = client.get_world()
    original_settings = sim_world.get_settings()
    settings = sim_world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.1
    sim_world.apply_settings(settings)
    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)
    for seq in sequences:
        try:
            world = World(sim_world, seq)
            sim_world.tick()
            while world.inter_frame < len(world.frames):
                print(world.inter_frame)
                world.refresh_traffic_flow()
                frame = sim_world.tick()
                world.tick(frame)
            if world is not None:
                world.destroy_all()

        except Exception as e:
            import traceback
            print('exception')
            print(traceback.format_exc())
            print(e)
        finally:
            print('destroying all actor')
            if world is not None:
                world.destroy_all()
    if original_settings:
        sim_world.apply_settings(original_settings)


def replay_one(args):
    world = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        sim_world = client.get_world()
        original_settings = sim_world.get_settings()
        settings = sim_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.10
        sim_world.apply_settings(settings)
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        world = World(sim_world, args.waymo_sequence)
        sim_world.tick()
        while world.inter_frame < len(world.frames):
            print(world.inter_frame)
            world.refresh_traffic_flow()
            frame = sim_world.tick()
            world.tick(frame)
    except Exception as e:
        import traceback
        print(traceback.format_exc())
        print(e)
    finally:
        print('destroying all actor')
        if original_settings:
            sim_world.apply_settings(original_settings)

        if world is not None:
            world.destroy_all()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
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
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--replay',
        metavar='type',
        default='one',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--waymo_sequence',
        default='segment-11070802577416161387_740_000_760_000', )
    argparser.add_argument(
        '--waymo_sequence_txt',
        default='/home/PJLAB/caixinyu/Documents/Waymo-Sim/src/config/sequence_replay.txt', )
    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)
    logging.info('listening to server %s:%s', args.host, args.port)

    if args.replay == 'one':
        replay_one(args)
    elif args.replay == 'txt':
        replay_all(args)


if __name__ == '__main__':
    main()
    print('finished')
