import traceback
import numpy as np
from Controller.waypoint_calculations import WaypointCalculations
from Controller.steer_controller import SteerController
import Util.Global as Global


def update(vehicle, waypoints_np, wp_distance, wp_interp, wp_interp_hash,lookahead_distance, closest_index, closest_distance):  
    
     
    current_location = vehicle.get_location()
    vehicle_transform = vehicle.get_transform()
    current_x = current_location.x 
    # current_y = abs(current_location.y - self.y_start)
    current_y =  - current_location.y  
    current_yaw = vehicle_transform.rotation.yaw
    
    Global.current_y= current_y
    Global.current_x= current_x

    if current_yaw >= 0:
        current_yaw -= 360
    tx,ty = WaypointCalculations.calculate_waypoint(closest_distance, closest_index,
        current_x, current_y, waypoints_np, lookahead_distance,
        wp_distance, wp_interp, wp_interp_hash)
    
    Global.yaw = current_yaw

    steer_output = SteerController.steer_control(current_yaw, current_x, current_y, tx, ty)
    return steer_output


