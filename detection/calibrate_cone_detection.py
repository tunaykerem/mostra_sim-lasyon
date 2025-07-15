#!/usr/bin/env python3

"""
Cone Detection Calibration Tool
Use this to find optimal HSV values for yellow cone detection
"""

import cv2
import numpy as np
from detection import cone_config

def nothing(x):
    pass

def main():
    # Initialize camera (you may need to change the camera index)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open camera")
        return
    
    # Create trackbars for HSV adjustment
    cv2.namedWindow('HSV Calibration')
    cv2.createTrackbar('H Min', 'HSV Calibration', cone_config.COLOR_DETECTION["lower_hsv"][0], 179, nothing)
    cv2.createTrackbar('S Min', 'HSV Calibration', cone_config.COLOR_DETECTION["lower_hsv"][1], 255, nothing)
    cv2.createTrackbar('V Min', 'HSV Calibration', cone_config.COLOR_DETECTION["lower_hsv"][2], 255, nothing)
    cv2.createTrackbar('H Max', 'HSV Calibration', cone_config.COLOR_DETECTION["upper_hsv"][0], 179, nothing)
    cv2.createTrackbar('S Max', 'HSV Calibration', cone_config.COLOR_DETECTION["upper_hsv"][1], 255, nothing)
    cv2.createTrackbar('V Max', 'HSV Calibration', cone_config.COLOR_DETECTION["upper_hsv"][2], 255, nothing)
    
    print("HSV Calibration Tool")
    print("Adjust the trackbars to isolate yellow cones")
    print("Press 'q' to quit and save values")
    print("Press 's' to save current values to cone_config.py")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Get trackbar values
        h_min = cv2.getTrackbarPos('H Min', 'HSV Calibration')
        s_min = cv2.getTrackbarPos('S Min', 'HSV Calibration')
        v_min = cv2.getTrackbarPos('V Min', 'HSV Calibration')
        h_max = cv2.getTrackbarPos('H Max', 'HSV Calibration')
        s_max = cv2.getTrackbarPos('S Max', 'HSV Calibration')
        v_max = cv2.getTrackbarPos('V Max', 'HSV Calibration')
        
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)
        
        # Apply mask to original frame
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Find contours for visualization
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw contours and bounding boxes
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > cone_config.COLOR_DETECTION["min_contour_area"]:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f'Area: {int(area)}', (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Display images
        cv2.imshow('Original', frame)
        cv2.imshow('Mask', mask)
        cv2.imshow('Result', result)
        
        # Display current values
        info_text = f"HSV: [{h_min}, {s_min}, {v_min}] to [{h_max}, {s_max}, {v_max}]"
        cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            # Save current values
            print(f"Current HSV values:")
            print(f"Lower: [{h_min}, {s_min}, {v_min}]")
            print(f"Upper: [{h_max}, {s_max}, {v_max}]")
            print("Update these values in detection/cone_config.py")
            
            # Show how to update config
            print("\nTo update cone_config.py, change:")
            print(f'"lower_hsv": [{h_min}, {s_min}, {v_min}],')
            print(f'"upper_hsv": [{h_max}, {s_max}, {v_max}],')
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
