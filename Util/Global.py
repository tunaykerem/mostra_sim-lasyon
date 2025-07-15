import numpy as np
import csv

# hız için
current_speed=0
v_desired=3.0
breaking=0
x_main,y_main=[],[]  # Initialize as empty lists for path points


#steer için
alpha = 0
alpha_hat = 0
math_sin_alpha = 0
steer_output = 0
yaw = -90


current_y=0
current_x=0

# Vehicle position and orientation tracking
vehicle_location = (0, 0, 0)  # (x, y, z)
vehicle_direction = "K"  # Direction K=North, G=South, D=East, B=West

direction = "K"
stop_x=0
stop_y=0
dur=0
time=0
# stop_points=[[0,15.5],[-72.5,36],[-27,-18]]
stop_points=[[]]
# stop_points=[[-48,75.5],[-72.5,36],[-27,-18]]
state="sabit"
durmacontorl=0
distance1=10
bekle_control=True
time_start=0
time_end=0

# Initialize obstacle avoidance variables
dubaVar = False  # Flag to indicate if an obstacle is detected
dubaEngel = ()   # Obstacle data (left_point, right_point)
obstacle_avoidance_active = False  # Flag to track if we're currently avoiding an obstacle
fallback_path_active = False  # Flag to indicate if we're using a fallback avoidance path
avoidance_active = False  # Flag to indicate we're in active avoidance mode
avoidance_path = []  # Store the current avoidance path for debugging
all_cones = []  # Store all detected cones

# Initialize grid matrix for path planning
# Create a simple default matrix (20x20 grid with no obstacles)
# This will be used by the obstacle avoidance system
matrix = np.zeros((20, 20), dtype=int)  # Default grid for path planning
mission = [[10, 10]]  # Default target position for path planning

def csv_to_list():
    data_list = []
    try:
        with open("Util/old_waypointss.csv", 'r') as file:
            csv_reader = csv.reader(file)
            next(csv_reader)

            for row in csv_reader:
                data_list.append(row)
    except Exception as e:
        print(f"Error reading waypoints CSV: {e}")
        # Return empty list if file not found or other error
    return data_list

def update_vehicle_position(x, y, z=0):
    """Update the vehicle's current position"""
    global vehicle_location, current_x, current_y
    vehicle_location = (x, y, z)
    current_x = x
    current_y = y
    
def update_vehicle_direction(dir_value):
    """Update the vehicle's current direction"""
    global vehicle_direction, direction
    vehicle_direction = dir_value
    direction = dir_value
    
def generate_default_avoidance_path(obstacle_point, side_offset=1.5):
    """Generate a simple default avoidance path when main algorithm fails
    
    Args:
        obstacle_point: The center point of the obstacle [x, y, radius]
        side_offset: How far to the side to avoid (positive=right, negative=left)
    
    Returns:
        x_points, y_points: Lists of x and y coordinates for the avoidance path
    """
    # Extract obstacle location
    obstacle_x, obstacle_y = obstacle_point[0], obstacle_point[1]
    
    # Create a simple path that goes around the obstacle
    # Default behavior is to go to the right of the obstacle
    
    # For North direction (K)
    if direction == "K":
        x_points = [current_x, current_x + side_offset, 
                   current_x + side_offset, current_x]
        y_points = [current_y, 
                   obstacle_y - 1.0,  # Before obstacle 
                   obstacle_y + 3.0,  # After obstacle
                   obstacle_y + 6.0]  # Back to original path
    
    # Add other directions as needed
    else:
        # Default fallback for any other direction
        x_points = [current_x, current_x + side_offset, 
                   current_x + side_offset, current_x]
        y_points = [current_y, 
                   obstacle_y - 1.0, 
                   obstacle_y + 3.0, 
                   obstacle_y + 6.0]
    
    return x_points, y_points
