import cv2
from setup import setup
from webcam1 import TrafficSignDetector
from Util import Global
from Controller import speed_controller
from Controller import lateral_controller as lc
from Util import Referans,State
import carla
from ultralytics import YOLO
import torch
import threading

from FinalPathGenerator import Levha
from FinalPathGenerator import setLabel
from FinalPathGenerator import PathGenerator as pg
from FinalPathGenerator import Main
from Util import Global
from matplotlib import pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np
from Controller.waypoint_interpolation import WaypointInterpolation
import detection.detect as dt1
import time
import socket
import struct
import pickle
vehicle, camera, camera_data = setup()
s_c = speed_controller.DrivingController() # speed controller

INTERP_LOOKAHEAD_DISTANCE = 1
INTERP_DISTANCE_RES       = 0.01 
closest_index = 0  
closest_distance = 0

stop_event = threading.Event()

Global.x_main,Global.y_main= Main.getFinalTrajectory()
Global.stop_points=[[Global.x_main[-1],Global.y_main[-1]]]


def stop_threads():
    camera.stop()
    camera.destroy()
    vehicle.destroy()
    stop_event.set()


def thread_function_1():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('localhost', 8485))
    while not stop_event.is_set():

        frame= camera_data['image'][:,:, :-1]

        data = pickle.dumps(frame)
        message_size = struct.pack("L", len(data))
        client_socket.sendall(message_size + data)
        # Sunucudan tespit sonuçlarını al
        result_size_data = client_socket.recv(8)
        if not result_size_data:
            break
        result_size = struct.unpack("Q", result_size_data)[0]
        result_data = b""
        while len(result_data) < result_size:
            result_data += client_socket.recv(4096)
        labels = pickle.loads(result_data)
        print("Received labels:", labels)
        key = cv2.waitKey(1)
        
        #simulasyonu durdurma
        if key == ord('q'):
            print("Q tuşuna basıldı, araç ve kamera siliniyor...")
            camera.stop()
            camera.destroy()
            vehicle.destroy()
            break

def thread_function_2():
    while not stop_event.is_set():
        #control şeyleri
        waypoints_np = np.array(list(zip(Global.x_main, Global.y_main)))
        wp_interp, wp_distance, wp_interp, wp_interp_hash = WaypointInterpolation.interpolate_waypoints(waypoints_np, INTERP_DISTANCE_RES)
        location = vehicle.get_location()
        velocity = vehicle.get_velocity()
        transform = vehicle.get_transform()
        throttle = s_c.update(vehicle)
        steer = lc.update(vehicle,waypoints_np,wp_distance,wp_interp,wp_interp_hash,INTERP_LOOKAHEAD_DISTANCE,closest_index,closest_distance)
        vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer,brake=Global.breaking))
        # Levha.levha(levha,Global.direction,Global.current_x,Global.current_y)   
        #setLabel.set_label(levha)
        State.set_state()
        Referans.set_referance()
   
cv2.destroyAllWindows()
# Thread'leri tanımlama
thread1 = threading.Thread(target=thread_function_1)
thread2 = threading.Thread(target=thread_function_2)

# Thread'leri başlatma
thread1.start()
thread2.start()


# Kullanıcının bir komutla thread'leri durdurmasını sağlamak için bekleme
input("Thread'leri durdurmak için Enter'a basın...\n")
stop_threads()

# Thread'lerin bitmesini bekleme (Bu durumda sonsuza kadar beklerler çünkü while True kullanıldı)
thread1.join()
thread2.join()
