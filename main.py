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

from FinalPathGenerator import Levha
from FinalPathGenerator import setLabel
from FinalPathGenerator import PathGenerator as pg
from FinalPathGenerator import Main
from Util import Global
from matplotlib import pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np
from Controller.waypoint_interpolation import WaypointInterpolation
import detection.detect as dt1 #detect yazıyodu burda
import time

detector = dt1.YOLODetector()
detectorv8 = TrafficSignDetector('solsag1.pt', 'no_left_turn_template.jpg', 'no_right_turn_template.jpg')

vehicle, camera, camera_data = setup()
s_c = speed_controller.DrivingController() # speed controller

INTERP_LOOKAHEAD_DISTANCE = 1
INTERP_DISTANCE_RES       = 0.01 
closest_index = 0  
closest_distance = 0


Global.x_main,Global.y_main= Main.getFinalTrajectory()
Global.stop_points=[[Global.x_main[-1],Global.y_main[-1]]]
# Global.stop_points=[[0,25],[Global.x_main[-1],Global.y_main[-1]]]
# plt.plot(Global.x_main, Global.y_main, marker='o', linestyle='-')
# plt.title('Rotaa')
# plt.xlabel('X Ekseni')
# plt.ylabel('Y Ekseni')
# plt.xlim(-90, 10)
# plt.ylim(-10, 90)
# plt.grid(True)
# plt.show()

while True:
    #kamera işlemleri
    frame= camera_data['image'][:,:, :-1]
    start_time = time.time()
    #Yolo V4
    frame,levha =  detector.process_frame(frame)
    #YOLO V8
    # frame,levha =detectorv8.process_frame(frame)
    elapsed_time = time.time() - start_time
    print("işlem süresi: ",elapsed_time)
    cv2.imshow('RGB Camera', frame)
    
    print(levha)
    key = cv2.waitKey(1)
    
    #simulasyonu durdurma
    if key == ord('q'):
        print("Q tuşuna basıldı, araç ve kamera siliniyor...")
        camera.stop()
        camera.destroy()
        vehicle.destroy()
        break

    #control şeyleri
    waypoints_np = np.array(list(zip(Global.x_main, Global.y_main)))
    wp_interp, wp_distance, wp_interp, wp_interp_hash = WaypointInterpolation.interpolate_waypoints(waypoints_np, INTERP_DISTANCE_RES)
    location = vehicle.get_location()
    velocity = vehicle.get_velocity()
    transform = vehicle.get_transform()
    throttle = s_c.update(vehicle)
    steer = lc.update(vehicle,waypoints_np,wp_distance,wp_interp,wp_interp_hash,INTERP_LOOKAHEAD_DISTANCE,closest_index,closest_distance)
    vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer,brake=Global.breaking))
    Levha.levha(levha,Global.direction,Global.current_x,Global.current_y)   
    #setLabel.set_label(levha)
    State.set_state()
    Referans.set_referance()
    
    # print(f'X: {location.x}, Y: {location.y}, Yaw: {transform.rotation.yaw}, Speed: { Global.current_speed} m/s')

cv2.destroyAllWindows()