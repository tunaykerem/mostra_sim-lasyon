import numpy as np
import math
import Util.Global as Global
import time
from Controller.yon import get_direction

class WaypointCalculations:

    @staticmethod
    def calculate_waypoint(closest_distance, closest_index, current_x, current_y, waypoints_np, lookahead_distance, wp_distance, wp_interp, wp_interp_hash):

        closest_distance = np.linalg.norm(np.array([
            waypoints_np[closest_index, 0] - current_x,
            waypoints_np[closest_index, 1] - current_y]))

        new_distance = closest_distance
        new_index = closest_index
        uzakliklar = np.sqrt((waypoints_np[:, 0] - current_x) ** 2 + (waypoints_np[:, 1] - current_y) ** 2)
        closest_index = np.argmin(uzakliklar)

        waypoint_subset_first_index = closest_index - 1

        if waypoint_subset_first_index < 0:
            waypoint_subset_first_index = 0

        waypoint_subset_last_index = closest_index

        total_distance_ahead = 0
        lookahead_distance=2
        while total_distance_ahead < lookahead_distance:
            total_distance_ahead += wp_distance[waypoint_subset_last_index]
            waypoint_subset_last_index += 1
            if waypoint_subset_last_index >= waypoints_np.shape[0]:
                waypoint_subset_last_index = waypoints_np.shape[0] - 1
                break

        new_waypoints = wp_interp[wp_interp_hash[waypoint_subset_first_index]: \
                      wp_interp_hash[waypoint_subset_last_index] + 1]


        waypoint_subset_first_index2 = closest_index - 1

        if waypoint_subset_first_index2 < 0:
            waypoint_subset_first_index2 = 0
        total_distance_ahead2 = 0
        lookahead_distance2=6
        waypoint_subset_last_index2 = closest_index

        while total_distance_ahead2 < lookahead_distance2:
            total_distance_ahead2 += wp_distance[waypoint_subset_last_index2]
            waypoint_subset_last_index2 += 1
            if waypoint_subset_last_index2 >= waypoints_np.shape[0]:
                waypoint_subset_last_index2 = waypoints_np.shape[0] - 1
                break

        new_waypoints2 = wp_interp[wp_interp_hash[waypoint_subset_first_index2]: \
                      wp_interp_hash[waypoint_subset_last_index2] + 1]
        

        # length = np.arange(0, 100, 1)
        # dx = [current_x - new_waypoints[icx][0] for icx in length]
        # dy = [current_y - new_waypoints[icy][1] for icy in length]
        # d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        # ind = d.index(min(d))
        tx = new_waypoints[-1][0]
        ty = new_waypoints[-1][1]

        tx2 = new_waypoints2[-1][0]
        ty2 = new_waypoints2[-1][1]
        get_direction()
        x_stop= Global.stop_points[0][0]
        y_stop= Global.stop_points[0][1]
        if Global.direction=="K":
            # pass

            x_main_last= x_stop
            y_main_last= y_stop

            if y_main_last-current_y>=0:
                if math.sqrt((x_main_last-tx2)**2 +(y_main_last-ty2)**2) <=0.5 and math.sqrt((x_main_last-tx2)**2 +(y_main_last-ty2)**2) >=0.0:
                    if(Global.dur != 2):
                        Global.dur=1 
                        Global.time= time.time()

        elif Global.direction=="B":

            x_main_last= x_stop
            y_main_last= y_stop

            if x_main_last-current_x<=0:
                if math.sqrt((x_main_last-tx2)**2 +(y_main_last-ty2)**2) <=0.5 and math.sqrt((x_main_last-tx2)**2 +(y_main_last-ty2)**2) >=0.0:
                    # print(current_x,current_y,math.sqrt((x_main_last-tx2)**2 +(y_main_last-ty2)**2))
                    if(Global.dur != 2):
                        Global.dur=1 
                        Global.time= time.time()
        elif Global.direction=="D":

            x_main_last= x_stop
            y_main_last= y_stop

            if x_main_last-current_x>=0:
                if math.sqrt((x_main_last-tx2)**2 +(y_main_last-ty2)**2) <=0.5 and math.sqrt((x_main_last-tx2)**2 +(y_main_last-ty2)**2) >=0.0:
                    # print(current_x,current_y,math.sqrt((x_main_last-tx2)**2 +(y_main_last-ty2)**2))
                    if(Global.dur != 2):
                        Global.dur=1 
                        Global.time= time.time()

        # elif Global.direction=="G":
        #     Global.stop_x=-63
        #     Global.stop_y=49
        #     x_main_last= -63
        #     y_main_last= 49
        #     if y_main_last-current_y<=0:

        #         if math.sqrt((x_main_last-tx2)**2 +(y_main_last-ty2)**2) <=0.5 and math.sqrt((x_main_last-tx2)**2 +(y_main_last-ty2)**2) >=0.0:
        #             # print(current_x,current_y,math.sqrt((x_main_last-tx2)**2 +(y_main_last-ty2)**2))
        #             print("Bitiş")

        #             Global.dur=1 
        #             Global.time= time.time()

        elif Global.direction=="G":
            # Global.stop_x=-30
            # Global.stop_y=-5
            # x_main_last= -30
            # y_main_last= -7
            # tx2=Global.x_main[-1]
            # ty2=Global.y_main[-1]

            x_main_last= x_stop
            y_main_last= y_stop

            if y_main_last-current_y<=0:

                if math.sqrt((x_main_last-tx2)**2 +(y_main_last-ty2)**2) <=0.5 and math.sqrt((x_main_last-tx2)**2 +(y_main_last-ty2)**2) >=0.0:
                    # print(current_x,current_y,math.sqrt((x_main_last-tx2)**2 +(y_main_last-ty2)**2))

                    if(Global.dur != 2):
                        Global.dur=2 
                        Global.time= time.time()

                    
                    
        return tx,ty