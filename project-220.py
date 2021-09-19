import glob
import os
import sys
import time
import random
import math

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

actor_list = []


def generate_lidar_blueprint(blueprint_library):
    lidar_blueprint = blueprint_library.find('sensor.lidar.ray_cast_semantic')
    lidar_blueprint.set_attribute('channels', str(64))
    lidar_blueprint.set_attribute('points_per_second', str(56000))
    lidar_blueprint.set_attribute('rotation_frequency', str(40))
    lidar_blueprint.set_attribute('range', str(100))
    return lidar_blueprint


object_id = {"None": 0,
             "Buildings": 1,
             "Fences": 2,
             "Other": 3,
             "Pedestrians": 4,
             "Poles": 5,
             "RoadLines": 6,
             "Roads": 7,
             "Sidewalks": 8,
             "Vegetation": 9,
             "Vehicles": 10,
             "Wall": 11,
             "TrafficsSigns": 12,
             "Sky": 13,
             "Ground": 14,
             "Bridge": 15,
             "RailTrack": 16,
             "GuardRail": 17,
             "TrafficLight": 18,
             "Static": 19,
             "Dynamic": 20,
             "Water": 21,
             "Terrain": 22
             }
key_list = list(object_id.keys())
val_list = list(object_id.values())


def semantic_lidar_data1(point_cloud_data):
    distance_name_data = {}
    for detection in point_cloud_data:
        position = val_list.index(detection.object_tag)
        distance = math.sqrt((detection.point.x ** 2) + (detection.point.y ** 2) + (detection.point.z ** 2))
        distance_name_data["distance"] = distance
        distance_name_data["name"] = key_list[position]
        if distance_name_data['name'] == 'Vehicles' and distance_name_data["distance"] > 3 and distance_name_data["distance"] < 8:#write condition here
            dropped_vehicle.apply_control(carla.VehicleControl(hand_brake=True))
            dropped_vehicle.set_light_state(carla.VehicleLightState(carla.VehicleLightState.Brake |
                                                                        carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.LowBeam))

            dropped_vehicle.apply_control(carla.VehicleControl(throttle=0.3, steer=0.2))
            time.sleep(1)
            dropped_vehicle.apply_control(carla.VehicleControl(throttle=0.3, steer=-0.2))
            time.sleep(1)
            car_control()
        else:
            continue



def car_control():
    dropped_vehicle.apply_control(carla.VehicleControl(throttle=0.51))

    time.sleep(20)


try:
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    map = world.get_map()
    get_blueprint_of_world = world.get_blueprint_library()
    car_model = get_blueprint_of_world.filter('model3')[0]
    spawn_point = (world.get_map().get_spawn_points()[20])
    dropped_vehicle = world.spawn_actor(car_model, spawn_point)

    police_car_blueprint = get_blueprint_of_world.filter('police')[0]
    police_car_spawn_point=world.get_map().get_spawn_points()[15]# set spawn point here
    police_car= world.spawn_actor(police_car_blueprint,police_car_spawn_point)# pass police_car_blueprint and police_Car_spawn_point
    police_car.apply_control(carla.VehicleControl(throttle=0.5))# write driving instruction to with throttle
    police_car.apply_control(carla.VehicleControl(throttle=0.5,steer=-0.2))# write driving instruction to with throttle and steer


    simulator_camera_location_rotation = carla.Transform(police_car_spawn_point.location, police_car_spawn_point.rotation)
    simulator_camera_location_rotation.location += spawn_point.get_forward_vector() * 30
    simulator_camera_location_rotation.rotation.yaw += 180
    simulator_camera_view = world.get_spectator()
    simulator_camera_view.set_transform(simulator_camera_location_rotation)
    actor_list.append(dropped_vehicle)
    actor_list.append(police_car)


    lidar_sensor = generate_lidar_blueprint(get_blueprint_of_world)
    sensor_lidar_spawn_point = carla.Transform(carla.Location(x=0, y=0, z=2.0),
                                               carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000))
    sensor = world.spawn_actor(lidar_sensor, sensor_lidar_spawn_point, attach_to=dropped_vehicle)

    sensor.listen(lambda data2: semantic_lidar_data1(data2))
    car_control()
    actor_list.append(sensor)

    time.sleep(1000)
finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')
