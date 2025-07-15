#!/usr/bin/env python3

"""
Yellow Cone Detection using Color-based Computer Vision
"""

import numpy as np
import cv2
from Util import Global
import math
from detection import cone_config

class ConeDetector:
    def __init__(self):
        # Load configuration
        self.config = cone_config
        
        # HSV range for yellow cones
        color_config = self.config.COLOR_DETECTION
        self.lower_yellow = np.array(color_config["lower_hsv"])
        self.upper_yellow = np.array(color_config["upper_hsv"])
        
        # Detection parameters
        self.min_contour_area = color_config["min_contour_area"]
        self.focal_length = color_config["focal_length"]
        self.real_cone_height = color_config["cone_height"]
        
        # Obstacle parameters
        obstacle_config = self.config.OBSTACLE_CONFIG
        self.cone_width = obstacle_config["cone_width"]
        self.max_distance = obstacle_config["max_detection_distance"]
        self.min_distance = obstacle_config["min_detection_distance"]
        
        # Debug settings
        self.debug = self.config.DEBUG
        
    def detect_cones(self, frame):
        """
        Detect yellow cones in the given frame
        Returns list of cone positions in camera coordinates
        """
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for yellow color
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        cone_positions = []
        detected_frame = frame.copy()
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by minimum area
            if area > self.min_contour_area:
                # Get bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate cone center
                center_x = x + w // 2
                center_y = y + h
                
                # Estimate distance using cone height
                distance = self.estimate_distance(h)
                
                # Convert to camera coordinates (meters)
                # Assuming center of image is straight ahead
                frame_height, frame_width = frame.shape[:2]
                
                # Horizontal displacement from center (negative = left, positive = right)
                horizontal_offset = (center_x - frame_width // 2) / frame_width * 2.0
                
                # Convert to world coordinates relative to vehicle
                cone_x = distance * horizontal_offset  # Side offset
                cone_y = distance  # Forward distance
                
                cone_positions.append([cone_x, cone_y])
                
                # Draw detection on frame
                cv2.rectangle(detected_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(detected_frame, f'Cone: {distance:.1f}m', 
                           (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.circle(detected_frame, (center_x, center_y), 5, (255, 0, 0), -1)
        
        return cone_positions, detected_frame
    
    def estimate_distance(self, pixel_height):
        """
        Estimate distance to cone based on its pixel height
        """
        if pixel_height > 0:
            distance = (self.real_cone_height * self.focal_length) / pixel_height
            return max(self.min_distance, min(self.max_distance, distance))
        return self.max_distance
    
    def update_obstacle_data(self, cone_positions):
        """
        Update Global.dubaEngel with detected cone positions
        Returns True if obstacle status changed (need trajectory update)
        """
        # Get previous state safely
        prev_obstacle_state = getattr(Global, 'dubaVar', False)
        prev_obstacle_data = getattr(Global, 'dubaEngel', ())
        
        if len(cone_positions) > 0:
            # Find the closest cone
            closest_cone = min(cone_positions, key=lambda pos: pos[1])  # Sort by forward distance
            
            # For obstacle avoidance, we need left and right boundaries
            # Assume cone width from configuration
            left_point = [closest_cone[1], closest_cone[0] - self.cone_width/2, 0.3]  # [forward, side, height]
            right_point = [closest_cone[1], closest_cone[0] + self.cone_width/2, 0.3]
            
            # Update global obstacle data
            Global.dubaEngel = (left_point, right_point)
            Global.dubaVar = True
            
            if self.debug["show_detection_info"]:
                print(f"Cone detected at: forward={closest_cone[1]:.2f}m, side={closest_cone[0]:.2f}m")
                print(f"Updated dubaEngel: {Global.dubaEngel}")
            
            # Check if obstacle state changed
            obstacle_changed = (not prev_obstacle_state or prev_obstacle_data != Global.dubaEngel)
            return obstacle_changed
        else:
            # No cones detected, clear obstacle data
            Global.dubaEngel = ()
            Global.dubaVar = False
            
            # Check if obstacle state changed (was there an obstacle before?)
            obstacle_changed = prev_obstacle_state
            if obstacle_changed and self.debug["show_detection_info"]:
                print("Obstacle cleared - trajectory update needed")
            return obstacle_changed
    
    def process_frame(self, frame):
        """
        Main processing function - detects cones and updates obstacle data
        Returns (processed_frame, obstacle_status_changed)
        """
        cone_positions, detected_frame = self.detect_cones(frame)
        obstacle_status_changed = self.update_obstacle_data(cone_positions)
        
        # Add status text to frame
        status_text = f"Cones detected: {len(cone_positions)}"
        if Global.dubaVar:
            status_text += " | OBSTACLE ACTIVE"
            cv2.putText(detected_frame, "OBSTACLE DETECTED", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        cv2.putText(detected_frame, status_text, 
                   (10, detected_frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return detected_frame, obstacle_status_changed
