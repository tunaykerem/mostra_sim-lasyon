from matplotlib import pyplot as plt
from matplotlib.colors import ListedColormap
from util import Global
import Path
from NesneTespit import buyukDuba, Donulmez
import numpy as np
import traceback

def getFinalTrajectory():
    """Generate the final trajectory based on mission points"""
    # Get current vehicle position
    vehicle_x, vehicle_y = Global.vehicle_location
    x_main = [vehicle_x]
    y_main = [vehicle_y]

    # Get data from CSV
    data = Global.csv_to_list()

    # Generate path through all mission points
    for mission in Global.mission:
        vehicle_x, vehicle_y = x_main[-1], y_main[-1]
        target_index_x, target_index_y = mission  # Get mission indices
        x_main, y_main = Path.getTrajectory(x_main, y_main, vehicle_x, vehicle_y, target_index_x, target_index_y)
    
    # Update global path
    Global.x_main = x_main
    Global.y_main = y_main

    return x_main, y_main

def escapeObstacle():
    """Generate an obstacle avoidance path"""
    # Initialize with default values in case of error
    vehicle_x, vehicle_y = Global.vehicle_location
    x_main = [vehicle_x]
    y_main = [vehicle_y]

    try:
        # Get data from CSV
        data = Global.csv_to_list()
        
        # Check if Global.dubaEngel is properly set
        if not hasattr(Global, 'dubaEngel') or Global.dubaEngel is None or len(Global.dubaEngel) < 2:
            print("⚠️ Warning: dubaEngel is not properly set")
            print("⚠️ Using default path (no obstacle)")
            return x_main, y_main
            
        # Call dubaEngel to generate avoidance path
        print("🚧 Obstacle detected! Generating avoidance trajectory...")
        print(f"🚧 Obstacle data: {Global.dubaEngel}")
        
        # Try to get avoidance path
        try:
            x_main, y_main = buyukDuba.dubaEngel()
            print(f"✅ Generated avoidance path with {len(x_main)} points")
            
            # Validate that the path is valid
            if not x_main or len(x_main) < 2 or not y_main or len(y_main) < 2:
                print("⚠️ Warning: Generated path is too short or invalid")
                x_main, y_main = generate_simple_fallback_path(vehicle_x, vehicle_y, Global.vehicle_direction)
                print(f"✅ Using fallback path with {len(x_main)} points")
        except Exception as e:
            print(f"❌ Error in buyukDuba.dubaEngel(): {e}")
            print(f"❌ Traceback: {traceback.format_exc()}")
            # Use a simple avoidance path as fallback
            print("⚠️ Using simple fallback avoidance path")
            x_main, y_main = generate_simple_fallback_path(vehicle_x, vehicle_y, Global.vehicle_direction)

        # Debug print matrix if available
        if hasattr(Global, 'matrix') and Global.matrix is not None:
            print("Current planning matrix:")
            print(Global.matrix)
        
        # Continue path to mission targets
        if hasattr(Global, 'mission') and Global.mission:
            for mission in Global.mission:
                try:
                    vehicle_x, vehicle_y = x_main[-1], y_main[-1]
                    target_index_x, target_index_y = mission  # Get mission indices
                    x_temp, y_temp = Path.getTrajectory(x_main, y_main, vehicle_x, vehicle_y, target_index_x, target_index_y)
                    
                    # Validate the new trajectory
                    if x_temp and len(x_temp) > 0 and y_temp and len(y_temp) > 0:
                        x_main, y_main = x_temp, y_temp
                    else:
                        print("⚠️ Warning: Path.getTrajectory returned invalid path, keeping original")
                except Exception as e:
                    print(f"❌ Error in Path.getTrajectory: {e}")
                    # Keep the existing path if there's an error
        else:
            print("⚠️ Warning: mission not properly set")

        # Update global path
        Global.x_main = x_main
        Global.y_main = y_main
        
        print(f"✅ Final avoidance path contains {len(x_main)} points")
        return x_main, y_main
        
    except Exception as e:
        print(f"❌ Error in escapeObstacle: {e}")
        print(f"❌ Traceback: {traceback.format_exc()}")
        # In case of any error, return a simple path and continue
        x_main, y_main = generate_simple_fallback_path(vehicle_x, vehicle_y, Global.vehicle_direction)
        Global.x_main = x_main
        Global.y_main = y_main
        return x_main, y_main

def generate_simple_fallback_path(vehicle_x, vehicle_y, direction):
    """Generate a simple path for emergency fallback"""
    lateral_offset = 2.0  # Move 2 meters to the side
    
    x_points = [vehicle_x]
    y_points = [vehicle_y]
    
    if direction == "K":  # North
        # Move to the right to avoid the obstacle
        x_points.extend([vehicle_x + lateral_offset, vehicle_x + lateral_offset, vehicle_x])
        y_points.extend([vehicle_y + 2, vehicle_y + 6, vehicle_y + 10])
    elif direction == "G":  # South
        # Move to the left to avoid the obstacle
        x_points.extend([vehicle_x - lateral_offset, vehicle_x - lateral_offset, vehicle_x])
        y_points.extend([vehicle_y - 2, vehicle_y - 6, vehicle_y - 10])
    elif direction == "D":  # East
        # Move to the right to avoid the obstacle
        x_points.extend([vehicle_x + 2, vehicle_x + 6, vehicle_x + 10])
        y_points.extend([vehicle_y - lateral_offset, vehicle_y - lateral_offset, vehicle_y])
    elif direction == "B":  # West
        # Move to the right to avoid the obstacle
        x_points.extend([vehicle_x - 2, vehicle_x - 6, vehicle_x - 10])
        y_points.extend([vehicle_y + lateral_offset, vehicle_y + lateral_offset, vehicle_y])
    else:
        # Default fallback if direction is unknown
        x_points.extend([vehicle_x + 1, vehicle_x + 2, vehicle_x + 3])
        y_points.extend([vehicle_y + 1, vehicle_y + 2, vehicle_y + 3])
    
    print(f"Generated simple fallback path with {len(x_points)} points")
    return x_points, y_points

def solaDonulmez():
    """Handle 'No Left Turn' sign detection"""
    vehicle_x, vehicle_y = Global.vehicle_location
    x_main = [vehicle_x]
    y_main = [vehicle_y]
    data = Global.csv_to_list()
        
    Donulmez.SolaDonulmez()
    Global.sola_donulmez = False

    for mission in Global.mission:
        vehicle_x, vehicle_y = x_main[-1], y_main[-1]
        target_index_x, target_index_y = mission  # Get mission indices
        x_main, y_main = Path.getTrajectory(x_main, y_main, vehicle_x, vehicle_y, target_index_x, target_index_y)

    Global.x_main = x_main
    Global.y_main = y_main

    return x_main, y_main

def sagaDonulmez():
    """Handle 'No Right Turn' sign detection"""
    vehicle_x, vehicle_y = Global.vehicle_location
    x_main = [vehicle_x]
    y_main = [vehicle_y]
    data = Global.csv_to_list()
        
    Donulmez.SagaDonulmez()
    Global.saga_donulmez = False
    for mission in Global.mission:
        vehicle_x, vehicle_y = x_main[-1], y_main[-1]
        target_index_x, target_index_y = mission  # Get mission indices
        x_main, y_main = Path.getTrajectory(x_main, y_main, vehicle_x, vehicle_y, target_index_x, target_index_y)

    Global.x_main = x_main
    Global.y_main = y_main
    return x_main, y_main

def girilmez():
    """Handle 'No Entry' sign detection"""
    vehicle_x, vehicle_y = Global.vehicle_location
    x_main = [vehicle_x]
    y_main = [vehicle_y]
    data = Global.csv_to_list()
        
    Donulmez.Girilmez()
    Global.girilmez = False
    print(Global.matrix)
    for mission in Global.mission:
        vehicle_x, vehicle_y = x_main[-1], y_main[-1]
        target_index_x, target_index_y = mission  # Get mission indices
        x_main, y_main = Path.getTrajectory(x_main, y_main, vehicle_x, vehicle_y, target_index_x, target_index_y)

    Global.x_main = x_main
    Global.y_main = y_main
    return x_main, y_main
