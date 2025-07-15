from Util import Global
import math

def nearest_point():
    current_x, current_y =Global.current_x,Global.current_y
    nearest_point = None
    min_dist = float('inf')
    dist = 0
    for i in range(len(Global.x_main)):
        dist = math.sqrt((Global.x_main[i] - current_x) ** 2 + (Global.y_main[i] - current_y) ** 2)
        if dist < min_dist:
            min_dist = dist
            nearest_point = (Global.x_main[i], Global.y_main[i])
    return nearest_point


def next_point():
    nearest = nearest_point()
    for i in range(len(Global.x_main)):
        if Global.x_main[i] == nearest[0] and Global.y_main[i] == nearest[1]:
            return Global.x_main[i + 1], Global.y_main[i + 1]
    return None
        
def distance():
    nearest = nearest_point()
    next_point_result = next_point()
    dist = math.sqrt((nearest[0] - next_point_result[0]) ** 2 + (nearest[1] - next_point_result[1]) ** 2)
    return dist

def finaldistance():
    current_x=Global.current_x
    current_y=Global.current_y
    
    dist = math.sqrt((Global.stop_points[0][0] - current_x) ** 2 + (Global.stop_points[0][1] - current_y) ** 2)
    return dist

        

