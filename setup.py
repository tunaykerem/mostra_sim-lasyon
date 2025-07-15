
import os

import sys
import glob
try:
    sys.path.append(glob.glob('../../carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg')[0])
except IndexError:
    pass
import carla  
import numpy as np

def setup():
    
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    #weather = world.get_weather()
    #weather.sun_azimuth_angle = 0.0
    #weather.sun_altitude_angle = 90.0
    #weather.precipitation = 0.0
    #world.set_weather(weather)
    map = world.get_map()
    blueprint_library = world.get_blueprint_library()

    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')

    target_location = carla.Location(x=0, y=0, z=0.2)
    transform = carla.Transform(target_location, carla.Rotation(yaw=-90))
    vehicle = world.spawn_actor(vehicle_bp, transform)

    spectator = world.get_spectator()
    spectator_location = carla.Location(x=-29, y=9, z=37)
    spectator_transform = carla.Transform(spectator_location, carla.Rotation(yaw=270, pitch=-50))
    spectator.set_transform(spectator_transform)

    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_init_trans = carla.Transform(carla.Location(z=1.6, x=0.4))
    IM_WIDTH = 1920
    IM_HEIGHT = 1080
    camera_bp.set_attribute('image_size_x', str(IM_WIDTH))
    camera_bp.set_attribute('image_size_y', str(IM_HEIGHT))
    # Improve image quality with better FOV and higher quality settings
    camera_bp.set_attribute('fov', '100')  # Wider field of view
    camera_bp.set_attribute('lens_circle_falloff', '5.0')
    camera_bp.set_attribute('lens_circle_multiplier', '0.0')
    camera_bp.set_attribute('chromatic_aberration_intensity', '0.0')
    camera_bp.set_attribute('chromatic_aberration_offset', '0.0')
    
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)
    camera_data = {'image': np.zeros((IM_HEIGHT, IM_WIDTH, 4))}
    camera.listen(lambda image: camera_callback(image, camera_data))

    control = carla.VehicleControl()
    control.throttle = 0.5  
    vehicle.apply_control(control)
    
    return vehicle, camera, camera_data

def camera_callback(image, data_dict):
    data_dict['image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
