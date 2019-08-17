#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Spawn NPCs into the simulation"""

import glob
import os
import sys

# try:
#     sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
#         sys.version_info.major,
#         sys.version_info.minor,
#         'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
# except IndexError:
#     pass

import carla
from carla import ColorConverter

import argparse
import logging
import random
import queue

import numpy as np
from skimage.io import imsave


def carla_img_to_np(carla_img):
    carla_img.convert(ColorConverter.Raw)

    img = np.frombuffer(carla_img.raw_data, dtype=np.dtype('uint8'))
    img = np.reshape(img, (carla_img.height, carla_img.width, 4))
    img = img[:,:,:3]
    img = img[:,:,::-1]

    return img

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
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=100,
        type=int,
        help='number of vehicles (default: 10)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=200,
        type=int,
        help='number of walkers (default: 50)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='vehicles filter (default: "vehicle.*")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='pedestrians filter (default: "walker.pedestrian.*")')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    camera_list = []
    vehicle_id = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    
    # import pdb; pdb.set_trace()
    
    try:
        # client.reload_world()
        world = client.load_world('Town01')
        blueprints = world.get_blueprint_library().filter(args.filterv)
        blueprintsWalkers = world.get_blueprint_library().filter(args.filterw)

        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        for n, transform in enumerate(spawn_points):
            # transform = carla.Transform(carla.Location(x=322.790924, y=-2.114089, z=1.320535), carla.Rotation(pitch=0.000000, yaw=-179.999939, roll=0.000000))
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')
            batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))
            # batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, False)))


        for response in client.apply_batch_sync(batch):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        for i in range(len(vehicles_list)):
            vehicle_id.append(vehicles_list[i])

        vehicles = world.get_actors(vehicle_id)

        # -------------
        # Spawn Walkers
        # -------------
        # 1. take all the random locations to spawn
        spawn_points = []
        for i in range(args.number_of_walkers):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            # loc = carla.Location(x=330.790924, y=-2.114089, z=1.320535)

            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)
                
            print (loc)
        # 2. we spawn the walker object
        batch = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invencible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            batch.append(SpawnActor(walker_bp, spawn_point))
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id
        # 4. we put altogether the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        world.wait_for_tick()

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            # all_actors[i].go_to_location(carla.Location(x=314.790924, y=-2.114089, z=1.320535))
            # all_actors[i].go_to_location(loc+carla.Location(x=10.0))
            # random max speed
            all_actors[i].set_max_speed(1 + random.random())    # max speed between 1 and 2 (default is 1.4 m/s)

        print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))
        
        # # NOTE: This has to happen after walkers spawning!
        # batch = []
        # vehicle_controller_id = []
        # walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        # for i in range(len(vehicle_id)):
        #     batch.append(SpawnActor(walker_controller_bp, carla.Transform(), vehicle_id[i]))
        # results = client.apply_batch_sync(batch, True)
        # for i in range(len(results)):
        #     if results[i].error:
        #         logging.error(results[i].error)
        #     else:
        #         vehicle_controller_id.append(results[i].actor_id)

        # vehicle_controller = world.get_actors(vehicle_controller_id)

        for i in range(len(vehicles)):
            # start walker
            vehicles[i].start_dtcrowd()
            # set walk to random point
            # vehicle_controller[i].start()
            # vehicle_controller[i].go_to_location(world.get_random_location_from_navigation())
            # vehicle_controller[i].set_max_speed(1 + random.random())


        # Set camera
        rgb_queue = queue.Queue()
        rgb_camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        rgb_camera_bp.set_attribute('image_size_x', '800')
        rgb_camera_bp.set_attribute('image_size_y', '600')
        rgb_camera_bp.set_attribute('fov', '100')
        rgb_camera = world.spawn_actor(
            rgb_camera_bp,
            carla.Transform(carla.Location(x=-3.5, z=1.4), carla.Rotation(pitch=-15)),
            attach_to=all_actors[39])

        rgb_camera.listen(rgb_queue.put)
        camera_list.append(rgb_camera)
        
        for _ in range(20):
            world.wait_for_tick()
            
        with rgb_queue.mutex:
            rgb_queue.queue.clear()
        
        for t in range(2000):
            ts = world.wait_for_tick()
            rgb_image = carla_img_to_np(rgb_queue.get())
            imsave('tmp/test-%04d.jpg'%t, rgb_image)
            

    finally:

        # remove peds from dtcrowd
        for i in range(len(vehicle_id)):
            vehicles[i].stop_dtcrowd()

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        # stop walker controllers (list is [controler, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')

