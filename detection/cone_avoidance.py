import numpy as np
import math
from Util import Global

class ObstacleAvoidanceSystem:
    """
    Advanced system for obstacle avoidance (cones and walls) that builds on existing avoidance logic
    """
    def __init__(self):
        # Configuration parameters
        self.safety_margin = 1.5  # meters - increased for better avoidance
        self.cone_width = 0.8  # meters - increased for better detection
        self.wall_width = 2.0  # meters - width for wall obstacles
        self.avoidance_distance = 10.0  # meters - increased to detect obstacles earlier
        self.min_obstacle_distance = 1.5  # minimum distance to trigger avoidance - increased
        self.debug = True
        self.use_direct_avoidance = True  # Always use direct avoidance, bypassing complex planner
        
    def process_obstacle_detections(self, cone_positions, wall_detected=False):
        """
        Process obstacle positions (cones and walls) and determine if avoidance is needed
        Returns: (need_trajectory_update, obstacle_data)
        """
        # Store previous state to detect changes
        prev_obstacle_state = getattr(Global, 'dubaVar', False)
        prev_obstacle_data = getattr(Global, 'dubaEngel', ())
        
        # Handle wall detection
        if wall_detected:
            if self.debug:
                print("🧱 WALL DETECTED! Initiating avoidance maneuver...")
            
            # For wall, create obstacle data assuming wall is directly ahead
            wall_x, wall_y = 0, 3.0  # Assume wall is 3m ahead, centered
            
            # Calculate left and right boundary points of the wall
            left_point = [wall_y, wall_x - self.wall_width, 1.0]  # [forward, side, height]
            right_point = [wall_y, wall_x + self.wall_width, 1.0]
            
            # Add safety margin
            left_point[1] -= self.safety_margin
            right_point[1] += self.safety_margin
            
            # Set global obstacle data
            Global.dubaEngel = (left_point, right_point)
            Global.dubaVar = True
            
            if self.debug:
                print(f"🧱 Wall obstacle set: {Global.dubaEngel}")
            
            return True, (left_point, right_point)
        
        # Handle cone detection (existing logic)
        if not cone_positions or len(cone_positions) == 0:
            # No cones detected, clear obstacle data
            Global.dubaEngel = ()
            Global.dubaVar = False
            
            # Only return True if we're clearing a previous obstacle
            return prev_obstacle_state, ()
        
        # Find closest cone
        distances = [math.sqrt(x**2 + y**2) for x, y in cone_positions]
        closest_idx = distances.index(min(distances))
        closest_cone = cone_positions[closest_idx]
        closest_distance = distances[closest_idx]
        
        # Always store all cones for more comprehensive avoidance
        Global.all_cones = cone_positions
        
        # If cone is too far, ignore it
        if closest_distance > self.avoidance_distance:
            if self.debug:
                print(f"Cone detected but too far ({closest_distance:.2f}m > {self.avoidance_distance}m)")
            
            # Clear obstacle data if it was set
            if prev_obstacle_state:
                Global.dubaEngel = ()
                Global.dubaVar = False
                return True, ()
            return False, ()
        
        # Check if cone is close enough to require immediate action
        if closest_distance < self.min_obstacle_distance:
            if self.debug:
                print(f"⚠️ CONE VERY CLOSE! Distance: {closest_distance:.2f}m - IMMEDIATE AVOIDANCE NEEDED!")
        
        # Convert cone to obstacle data format expected by the existing system
        # Calculate left and right boundary points of the cone
        cone_x, cone_y = closest_cone
        
        # Calculate perpendicular direction to create width
        # The existing system expects format: [forward_dist, side_offset, height]
        # For better avoidance, make the obstacle wider
        left_point = [cone_y, cone_x - self.cone_width, 0.5]  # [forward, side, height]
        right_point = [cone_y, cone_x + self.cone_width, 0.5]
        
        # Add safety margin
        left_point[1] -= self.safety_margin
        right_point[1] += self.safety_margin
        
        # Set global obstacle data (used by the existing avoidance system)
        Global.dubaEngel = (left_point, right_point)
        Global.dubaVar = True
        
        if self.debug:
            print(f"🔴 Cone detected at: forward={cone_y:.2f}m, side={cone_x:.2f}m")
            print(f"📍 Updated dubaEngel: {Global.dubaEngel}")
        
        # Always return True to ensure the avoidance path is generated immediately
        return True, (left_point, right_point)
    
    def process_cone_detections(self, cone_positions):
        """
        Backward compatibility method - redirects to process_obstacle_detections
        """
        return self.process_obstacle_detections(cone_positions, wall_detected=False)
    
    def generate_avoidance_path(self, current_x, current_y, direction):
        """
        Generate a path to avoid the detected cone
        This will now ALWAYS use our direct avoidance method to ensure reliability
        """
        if not Global.dubaVar:
            # No obstacle to avoid
            print("No obstacle to avoid (dubaVar is False)")
            return None, None
        
        print(f"🚧 Generating avoidance path for obstacle at position {Global.dubaEngel}")
        print(f"🚗 Current position: x={current_x}, y={current_y}, direction={direction}")
        
        # Always use direct avoidance path for reliability
        if self.use_direct_avoidance:
            print("🛣️ Using direct avoidance path generation (bypassing complex planner)")
            return self._generate_direct_avoidance_path(current_x, current_y, direction)
            
        # Only reached if use_direct_avoidance is False (fallback to complex planner)    
        # Ensure Global.matrix is initialized
        if not hasattr(Global, 'matrix') or Global.matrix is None:
            print("Warning: matrix not found in Global.py, creating default matrix")
            Global.matrix = np.zeros((20, 20), dtype=int)
            
        # Ensure Global.mission is initialized
        if not hasattr(Global, 'mission') or Global.mission is None:
            print("Warning: mission not found in Global.py, creating default mission")
            Global.mission = [[10, 10]]
            
        # Import here to avoid circular imports
        try:
            from FinalPathGenerator.engelkacis import escapeFromObstacle
            
            # Use the existing escape from obstacle implementation
            escape_obj = escapeFromObstacle(current_x, current_y, direction)
            x_main, y_main = escape_obj.createNewTrajectory()
            
            # Debug: Print some points of the generated trajectory
            print(f"Avoidance trajectory generated with {len(x_main)} points")
            if len(x_main) > 5:
                print(f"First 3 points: ({x_main[0]}, {y_main[0]}), ({x_main[1]}, {y_main[1]}), ({x_main[2]}, {y_main[2]})")
                print(f"Last 3 points: ({x_main[-3]}, {y_main[-3]}), ({x_main[-2]}, {y_main[-2]}), ({x_main[-1]}, {y_main[-1]})")
            
            return x_main, y_main
        except Exception as e:
            print(f"❌ Error generating avoidance path with complex planner: {e}")
            print("🛣️ Falling back to direct avoidance path...")
            
            # Generate a simple avoidance path directly
            # This is a fallback in case the engelkacis.py system fails
            return self._generate_direct_avoidance_path(current_x, current_y, direction)
            
    def _generate_direct_avoidance_path(self, current_x, current_y, direction):
        """
        Generate a simple avoidance path directly without using engelkacis.py
        This is now our primary avoidance method for reliability
        """
        # Get obstacle position from dubaEngel
        if not Global.dubaEngel or len(Global.dubaEngel) < 2:
            print("❌ No valid obstacle data in dubaEngel")
            return None, None
            
        left_point, right_point = Global.dubaEngel
        
        # Calculate the obstacle's position from the left and right points
        obstacle_forward = (left_point[0] + right_point[0]) / 2
        obstacle_side = (left_point[1] + right_point[1]) / 2
        
        print(f"� Obstacle position: forward={obstacle_forward:.2f}m, side={obstacle_side:.2f}m")
        
        # MODIFIED: Always prioritize left-side avoidance (standard driving behavior)
        # -1 = left, 1 = right
        # Set to -1 (left) by default to prioritize proper lane-change behavior
        avoid_side = -1  # Always prefer to go left (into the passing lane)
        
        # Only go right if the obstacle is significantly to the left and going left isn't feasible
        # e.g., if the obstacle is at the left edge of the road
        if obstacle_side < -1.5:  # If obstacle is very far to the left
            print("🚗 Obstacle is far to the left, avoiding to the right as exception")
            avoid_side = 1
            
        print(f"🛣️ Avoiding to the {'left' if avoid_side == -1 else 'right'}")
        
        # Calculate lateral offset for avoidance (move to the side by 3.0 meters - increased for better lane change)
        lateral_offset = 3.0 * avoid_side
        
        # Get all detected cones for more comprehensive planning
        all_cones = getattr(Global, 'all_cones', [])
        
        # Define how far ahead to extend the path
        extend_distance = 15.0  # meters
        
        # Create a more detailed avoidance path with smooth transitions
        x_points = []
        y_points = []
        
        # Create a smooth path with more points
        if direction == "K":  # North
            # Starting point
            x_points.append(current_x)
            y_points.append(current_y)
            
            # Gradual avoidance curve (5 points)
            for i in range(1, 6):
                ratio = i / 5.0
                x_points.append(current_x + lateral_offset * ratio)  # lateral_offset is negative = LEFT
                y_points.append(current_y + 2 * ratio)
            
            # Continue straight on offset path
            x_points.append(current_x + lateral_offset)
            y_points.append(current_y + 6)
            
            # Gradual return curve (5 points)
            for i in range(1, 6):
                ratio = i / 5.0
                x_points.append(current_x + lateral_offset * (1 - ratio))
                y_points.append(current_y + 6 + 4 * ratio)
            
            # Continue forward on original path
            x_points.append(current_x)
            y_points.append(current_y + 12)
            
            # Extend path forward
            x_points.append(current_x)
            y_points.append(current_y + extend_distance)
            
        elif direction == "G":  # South
            # Starting point
            x_points.append(current_x)
            y_points.append(current_y)
            
            # Gradual avoidance curve (5 points)
            for i in range(1, 6):
                ratio = i / 5.0
                x_points.append(current_x + lateral_offset * ratio)  # lateral_offset is negative = LEFT
                y_points.append(current_y - 2 * ratio)
            
            # Continue straight on offset path
            x_points.append(current_x + lateral_offset)
            y_points.append(current_y - 6)
            
            # Gradual return curve (5 points)
            for i in range(1, 6):
                ratio = i / 5.0
                x_points.append(current_x + lateral_offset * (1 - ratio))
                y_points.append(current_y - 6 - 4 * ratio)
            
            # Continue forward on original path
            x_points.append(current_x)
            y_points.append(current_y - 12)
            
            # Extend path forward
            x_points.append(current_x)
            y_points.append(current_y - extend_distance)
            
        elif direction == "D":  # East
            # Starting point
            x_points.append(current_x)
            y_points.append(current_y)
            
            # Gradual avoidance curve (5 points)
            for i in range(1, 6):
                ratio = i / 5.0
                x_points.append(current_x + 2 * ratio)
                y_points.append(current_y + lateral_offset * ratio)  # lateral_offset is negative = LEFT
            
            # Continue straight on offset path
            x_points.append(current_x + 6)
            y_points.append(current_y + lateral_offset)
            
            # Gradual return curve (5 points)
            for i in range(1, 6):
                ratio = i / 5.0
                x_points.append(current_x + 6 + 4 * ratio)
                y_points.append(current_y + lateral_offset * (1 - ratio))
            
            # Continue forward on original path
            x_points.append(current_x + 12)
            y_points.append(current_y)
            
            # Extend path forward
            x_points.append(current_x + extend_distance)
            y_points.append(current_y)
            
        elif direction == "B":  # West
            # Starting point
            x_points.append(current_x)
            y_points.append(current_y)
            
            # Gradual avoidance curve (5 points)
            for i in range(1, 6):
                ratio = i / 5.0
                x_points.append(current_x - 2 * ratio)
                y_points.append(current_y + lateral_offset * ratio)  # lateral_offset is negative = LEFT
            
            # Continue straight on offset path
            x_points.append(current_x - 6)
            y_points.append(current_y + lateral_offset)
            
            # Gradual return curve (5 points)
            for i in range(1, 6):
                ratio = i / 5.0
                x_points.append(current_x - 6 - 4 * ratio)
                y_points.append(current_y + lateral_offset * (1 - ratio))
            
            # Continue forward on original path
            x_points.append(current_x - 12)
            y_points.append(current_y)
            
            # Extend path forward
            x_points.append(current_x - extend_distance)
            y_points.append(current_y)
        
        print(f"🛣️ Direct avoidance path generated with {len(x_points)} points")
        print(f"Path first points: {list(zip(x_points[:3], y_points[:3]))}")
        print(f"Path last points: {list(zip(x_points[-3:], y_points[-3:]))}")
        
        # Store the avoidance path for debugging
        Global.avoidance_path = list(zip(x_points, y_points))
        
        return x_points, y_points
