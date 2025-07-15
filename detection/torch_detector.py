#!/usr/bin/env python3

"""
Simple YOLOv8 Traffic Sign Detector for last.pt weights
No external dependencies beyond ultralytics and OpenCV
"""

import cv2
import numpy as np
from ultralytics import YOLO
import time

class SimpleYOLOv8Detector:
    def __init__(self, model_path='last.pt'):
        """
        Initialize YOLOv8 detector with specified model weights
        """
        self.model = YOLO(model_path)
        print(f"Simple YOLOv8 detector initialized with weights: {model_path}")
        
        # Get class names from the model
        self.class_names = self.model.model.names
        print(f"Available classes: {list(self.class_names.values())}")
    
    def process_frame(self, frame):
        """
        Process frame and return (annotated_frame, detected_label)
        Compatible with existing detection pipeline
        """
        start_time = time.time()
        detected_label = "None"
        
        # Ensure frame is in correct format
        if frame.dtype != np.uint8:
            frame = frame.astype(np.uint8)
        
        # Make a copy for annotation
        annotated_frame = frame.copy()
        
        try:
            # Run inference
            results = self.model(frame, verbose=False)
            
            best_confidence = 0
            best_detection = None
            
            for result in results:
                if hasattr(result, 'boxes') and result.boxes is not None:
                    boxes = result.boxes
                    
                    # Process each detection
                    for i in range(len(boxes)):
                        # Get detection data
                        box = boxes.xyxy[i].cpu().numpy()  # x1, y1, x2, y2
                        confidence = float(boxes.conf[i].cpu().numpy())
                        class_id = int(boxes.cls[i].cpu().numpy())
                        
                        # Get class name
                        class_name = self.class_names.get(class_id, "Unknown")
                        
                        # Keep track of best detection
                        if confidence > best_confidence:
                            best_confidence = confidence
                            best_detection = {
                                'class_name': class_name,
                                'confidence': confidence,
                                'box': box,
                                'class_id': class_id
                            }
                        
                        # Draw bounding box
                        x1, y1, x2, y2 = box.astype(int)
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        
                        # Draw label
                        label = f"{class_name} {confidence:.2f}"
                        cv2.putText(annotated_frame, label, (x1, y1-10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        
                        # Handle cone detection if present
                        if class_name == "cone":
                            self._handle_cone_detection(box, frame.shape)
            
            # Set detected label to best detection
            if best_detection:
                detected_label = best_detection['class_name']
                print(f"YOLOv8 detected: {detected_label} (confidence: {best_detection['confidence']:.2f})")
        
        except Exception as e:
            print(f"YOLOv8 detection error: {e}")
            detected_label = "None"
        
        elapsed_time = time.time() - start_time
        
        return annotated_frame, detected_label
    
    def _handle_cone_detection(self, box, frame_shape):
        """
        Handle cone detection for obstacle avoidance
        """
        try:
            from Util import Global
            
            # Get bounding box coordinates
            x1, y1, x2, y2 = box.astype(int)
            
            # Calculate cone center and dimensions
            center_x = (x1 + x2) // 2
            center_y = y2  # Bottom of bounding box
            box_width = x2 - x1
            box_height = y2 - y1
            
            # Estimate distance (simple approximation)
            frame_height, frame_width = frame_shape[:2]
            estimated_distance = max(1.0, min(15.0, (50.0 / max(box_height, 10)) * 10))
            
            # Calculate relative position
            horizontal_offset = (center_x - frame_width // 2) / frame_width * 2.0
            cone_x = estimated_distance * horizontal_offset
            cone_y = estimated_distance
            
            # Update obstacle data
            cone_width = 0.3
            left_point = [cone_y, cone_x - cone_width/2, 0.3]
            right_point = [cone_y, cone_x + cone_width/2, 0.3]
            Global.dubaEngel = (left_point, right_point)
            Global.dubaVar = True
            
            print(f"YOLOv8 cone detected at: forward={cone_y:.2f}m, side={cone_x:.2f}m")
            
        except Exception as e:
            print(f"Error handling cone detection: {e}")
    
    def get_class_names(self):
        """
        Get available class names from the model
        """
        return self.class_names
