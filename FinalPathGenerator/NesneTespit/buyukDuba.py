import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from util import Global
import numpy as np
import FindCarIndex
from engelkacis import escapeFromObstacle

def en_yakin_nokta(given_point):
    """Find the closest point in the path to the given point"""
    min_mesafe = float('inf')
    en_yakin_nokta = None
    
    # Safely get main path
    if hasattr(Global, 'x_main') and hasattr(Global, 'y_main') and Global.x_main and Global.y_main:
        x_main = Global.x_main
        y_main = Global.y_main
    else:
        # Return default if no path exists
        return np.array([0, 0]), float('inf')
    
    # Tüm noktaları karşılaştır
    for x, y in zip(x_main, y_main):
        current_point = np.array([x, y])
        # Öklid mesafesini hesapla
        mesafe = np.linalg.norm(current_point - given_point)
        
        # En küçük mesafeyi ve noktayı güncelle
        if mesafe < min_mesafe:
            min_mesafe = mesafe
            en_yakin_nokta = current_point
    
    # If no closest point was found, return the origin with infinite distance
    if en_yakin_nokta is None:
        return np.array([0, 0]), float('inf')
        
    return en_yakin_nokta, min_mesafe


def dubaEngel():
    """Generate a path to avoid an obstacle (duba/cone)"""
    # Get vehicle position and direction (with safe defaults)
    vehicle_x = getattr(Global, 'vehicle_location', (0, 0, 0))[0]
    vehicle_y = getattr(Global, 'vehicle_location', (0, 0, 0))[1]
    vehicle_direction = getattr(Global, 'vehicle_direction', "K")
    
    # Initialize return variables with default values
    x_main = [vehicle_x]  # Start with vehicle location
    y_main = [vehicle_y]
    
    # Print debug info
    print(f"🚗 Vehicle at position ({vehicle_x}, {vehicle_y}), direction: {vehicle_direction}")
    print(f"🚧 Obstacle data: {Global.dubaEngel}")

    try:
        seritUzunlugu = 0
        
        # Check if dubaEngel is properly set
        if not hasattr(Global, 'dubaEngel') or Global.dubaEngel is None or len(Global.dubaEngel) < 2:
            print("⚠️ Warning: dubaEngel is not properly set, using fallback")
            # Generate a simple fallback path and return early
            x_main = generate_simple_avoidance_path(vehicle_x, vehicle_y, vehicle_direction, [0, 3])
            y_main = generate_simple_avoidance_path_y(vehicle_x, vehicle_y, vehicle_direction, [0, 3])
            return x_main, y_main
            
        left_point, right_point = Global.dubaEngel
        left_point = np.array(left_point)
        right_point = np.array(right_point)
        
        # Find the middle point of the obstacle
        merkez_nokta = (left_point + right_point) / 2
        print(f"📍 Calculated obstacle center point: {merkez_nokta}")
        
        # Calculate distance between two points
        mesafe = np.linalg.norm(left_point - right_point)
        
        # Calculate obstacle coordinates based on vehicle direction
        if vehicle_direction == "K":
            engel_x = merkez_nokta[1] + vehicle_x
            engel_y = merkez_nokta[0] + vehicle_y
            merkez_nokta = [engel_x, engel_y]
        elif vehicle_direction == "G":
            engel_x = merkez_nokta[1] + vehicle_x
            engel_y = vehicle_y - merkez_nokta[0]
            merkez_nokta = [engel_x, engel_y]
        elif vehicle_direction == "B":
            engel_x = vehicle_x - merkez_nokta[0]
            engel_y = vehicle_y + merkez_nokta[1]
            merkez_nokta = [engel_x, engel_y]
        elif vehicle_direction == "D":
            engel_x = vehicle_x + merkez_nokta[0]
            engel_y = vehicle_y + merkez_nokta[1]
            merkez_nokta = [engel_x, engel_y]  
        
        # Determine lane width based on distance
        if 2 <= mesafe <= 3.5:
            seritUzunlugu = 1
        elif 3.5 < mesafe <= 7.5:
            seritUzunlugu = 2
        else:
            seritUzunlugu = 0
            
        print(f"📏 Calculated lane width: {seritUzunlugu}, distance: {mesafe}")
        print(f"🎯 Final obstacle center: {merkez_nokta}")
        
        # Get waypoint data
        data = Global.csv_to_list()

        # Process based on lane width
        if seritUzunlugu == 1:
            closest_point, distance = en_yakin_nokta(merkez_nokta)
            if distance <= 1:
                try:
                    escape_obj = escapeFromObstacle(vehicle_x, vehicle_y, vehicle_direction)
                    x_main, y_main = escape_obj.createNewTrajectory()
                    print(f"✅ Generated complex avoidance trajectory with {len(x_main)} points")
                except Exception as e:
                    print(f"❌ Error in escapeFromObstacle: {e}")
                    # Fall back to simple avoidance
                    x_main = generate_simple_avoidance_path(vehicle_x, vehicle_y, vehicle_direction, merkez_nokta)
                    y_main = generate_simple_avoidance_path_y(vehicle_x, vehicle_y, vehicle_direction, merkez_nokta)
                    print(f"⚠️ Using fallback simple avoidance with {len(x_main)} points")
            else:
                # Simple avoidance for when the point is not close enough
                x_main = generate_simple_avoidance_path(vehicle_x, vehicle_y, vehicle_direction, merkez_nokta)
                y_main = generate_simple_avoidance_path_y(vehicle_x, vehicle_y, vehicle_direction, merkez_nokta)
                print(f"📍 Point not close enough (distance={distance}), using simple avoidance")
                
        elif seritUzunlugu == 2:
            closest_point, distance = en_yakin_nokta(merkez_nokta)
            if distance <= 2.2:
                try:
                    x_index, y_index = FindCarIndex.findcarindex(closest_point[0], closest_point[1], data)
                    index_coordinate = [x_index, y_index]
                    
                    # Process corner points if they exist
                    if hasattr(Global, 'corner_point') and Global.corner_point:
                        for point in Global.corner_point:
                            if point == index_coordinate:
                                if vehicle_direction == "K":
                                    x_index = point[0] - 1
                                    y_index = point[1]
                                elif vehicle_direction == "G":
                                    x_index = point[0] + 1
                                    y_index = point[1]
                                elif vehicle_direction == "B":
                                    x_index = point[0]
                                    y_index = point[1] - 1
                                elif vehicle_direction == "D":
                                    x_index = point[0]
                                    y_index = point[1] + 1
                    
                    # Check if matrix attribute exists and update it
                    if hasattr(Global, 'matrix') and Global.matrix is not None:
                        try:
                            Global.matrix[x_index, y_index] = 0
                            print(f"✅ Updated matrix at position [{x_index}, {y_index}]")
                        except Exception as e:
                            print(f"❌ Error updating matrix: {e}")
                except Exception as e:
                    print(f"❌ Error in findcarindex: {e}")
                
                # Generate a simple avoidance path
                x_main = generate_simple_avoidance_path(vehicle_x, vehicle_y, vehicle_direction, merkez_nokta)
                y_main = generate_simple_avoidance_path_y(vehicle_x, vehicle_y, vehicle_direction, merkez_nokta)
                print(f"⚠️ Using simple avoidance path for lane width 2")
            else:
                # Simple avoidance for when the point is not close enough
                x_main = generate_simple_avoidance_path(vehicle_x, vehicle_y, vehicle_direction, merkez_nokta)
                y_main = generate_simple_avoidance_path_y(vehicle_x, vehicle_y, vehicle_direction, merkez_nokta)
                print(f"📍 Point not close enough (distance={distance}), using simple avoidance")
        else:
            # For seritUzunlugu == 0, generate a simple avoidance path
            print("⚠️ Using simple avoidance path (seritUzunlugu = 0)")
            x_main = generate_simple_avoidance_path(vehicle_x, vehicle_y, vehicle_direction, merkez_nokta)
            y_main = generate_simple_avoidance_path_y(vehicle_x, vehicle_y, vehicle_direction, merkez_nokta)
    
    except Exception as e:
        print(f"❌ Error in dubaEngel: {e}")
        # Fallback to a very simple avoidance path
        print("⚠️ Using emergency fallback avoidance path")
        x_main = generate_simple_avoidance_path(vehicle_x, vehicle_y, vehicle_direction, [0, 3])
        y_main = generate_simple_avoidance_path_y(vehicle_x, vehicle_y, vehicle_direction, [0, 3])
    
    # Ensure we have valid return values
    if not x_main or len(x_main) == 0:
        print("⚠️ Empty x_main! Using vehicle position")
        x_main = [vehicle_x]
    if not y_main or len(y_main) == 0:
        print("⚠️ Empty y_main! Using vehicle position")
        y_main = [vehicle_y]
    
    print(f"✅ Returning avoidance path with {len(x_main)} points")
    return x_main, y_main

def generate_simple_avoidance_path(vehicle_x, vehicle_y, vehicle_direction, obstacle_point):
    """Generate a simple x-coordinates path to avoid an obstacle"""
    lateral_offset = 2.0  # Move 2 meters to the side
    
    if vehicle_direction == "K":  # North
        return [vehicle_x, vehicle_x + lateral_offset, vehicle_x + lateral_offset, vehicle_x]
    elif vehicle_direction == "G":  # South
        return [vehicle_x, vehicle_x - lateral_offset, vehicle_x - lateral_offset, vehicle_x]
    elif vehicle_direction == "D":  # East
        return [vehicle_x, vehicle_x + 2, vehicle_x + 6, vehicle_x + 10]
    elif vehicle_direction == "B":  # West
        return [vehicle_x, vehicle_x - 2, vehicle_x - 6, vehicle_x - 10]
    else:
        return [vehicle_x, vehicle_x + lateral_offset, vehicle_x]  # Default fallback

def generate_simple_avoidance_path_y(vehicle_x, vehicle_y, vehicle_direction, obstacle_point):
    """Generate a simple y-coordinates path to avoid an obstacle"""
    lateral_offset = 2.0  # Move 2 meters to the side
    
    if vehicle_direction == "K":  # North
        return [vehicle_y, vehicle_y + 2, vehicle_y + 6, vehicle_y + 10]
    elif vehicle_direction == "G":  # South
        return [vehicle_y, vehicle_y - 2, vehicle_y - 6, vehicle_y - 10]
    elif vehicle_direction == "D":  # East
        return [vehicle_y, vehicle_y - lateral_offset, vehicle_y - lateral_offset, vehicle_y]
    elif vehicle_direction == "B":  # West
        return [vehicle_y, vehicle_y + lateral_offset, vehicle_y + lateral_offset, vehicle_y]
    else:
        return [vehicle_y, vehicle_y + 2, vehicle_y + 6]  # Default fallback





