#!/usr/bin/env python3

"""
YOLOv8 Traffic Sign Detector Wrapper for last.pt weights
Compatible with the existing detection pipeline
"""

import cv2
import numpy as np
from ultralytics import YOLO
import supervision as sv
import time

class YOLOv8Detector:
    def __init__(self, model_path='last.pt'):
        """
        Initialize YOLOv8 detector with specified model weights
        """
        self.model = YOLO(model_path)
        # Fix BoxAnnotator initialization for compatibility
        try:
            self.box_annotator = sv.BoxAnnotator(thickness=2, text_thickness=1, text_scale=0.5)
        except TypeError:
            # Fallback for older supervision versions
            self.box_annotator = sv.BoxAnnotator(thickness=2)
        print(f"YOLOv8 detector initialized with weights: {model_path}")
        
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
            
            for result in results:
                if hasattr(result, 'boxes') and result.boxes is not None:
                    # Try different methods to create detections based on supervision version
                    try:
                        detections = sv.Detections.from_ultralytics(result)
                    except AttributeError:
                        try:
                            detections = sv.Detections.from_yolov8(result)
                        except AttributeError:
                            # Manual detection creation as fallback
                            boxes = result.boxes
                            if boxes is not None:
                                xyxy = boxes.xyxy.cpu().numpy()
                                confidence = boxes.conf.cpu().numpy()
                                class_id = boxes.cls.cpu().numpy().astype(int)
                                detections = sv.Detections(
                                    xyxy=xyxy,
                                    confidence=confidence,
                                    class_id=class_id
                                )
                            else:
                                continue
                    
                    if len(detections) > 0:
                        # Get the most confident detection
                        max_conf_idx = np.argmax(detections.confidence)
                        class_id = detections.class_id[max_conf_idx]
                        confidence = detections.confidence[max_conf_idx]
                        
                        # Get class name
                        class_name = self.class_names.get(class_id, "Unknown")
                        detected_label = class_name
                        
                        # Create labels for annotation
                        labels = [
                            f"{self.class_names.get(class_id, 'Unknown')} {conf:0.2f}"
                            for class_id, conf in zip(detections.class_id, detections.confidence)
                        ]
                        
                        # Annotate frame with error handling
                        try:
                            annotated_frame = self.box_annotator.annotate(
                                scene=annotated_frame, 
                                detections=detections, 
                                labels=labels
                            )
                        except Exception as e:
                            print(f"Annotation error: {e}")
                            # Fallback: draw simple rectangles
                            for i, (xyxy, conf, cls_id) in enumerate(zip(detections.xyxy, detections.confidence, detections.class_id)):
                                x1, y1, x2, y2 = xyxy.astype(int)
                                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                label = f"{self.class_names.get(cls_id, 'Unknown')} {conf:.2f}"
                                cv2.putText(annotated_frame, label, (x1, y1-10), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        
                        # Handle cone detection if present (check for both "cone" and "koni")
                        if class_name.lower() in ["cone", "koni"]:
                            self._handle_cone_detection(detections, max_conf_idx, frame.shape)
                        
                        print(f"YOLOv8 detected: {class_name} (confidence: {confidence:.2f})")
        
        except Exception as e:
            print(f"YOLOv8 detection error: {e}")
            detected_label = "None"
        
        elapsed_time = time.time() - start_time
        
        return annotated_frame, detected_label
    
    def _handle_cone_detection(self, detections, cone_idx, frame_shape):
        """
        Handle cone detection for obstacle avoidance
        """
        try:
            from Util import Global
            
            # Get bounding box
            bbox = detections.xyxy[cone_idx]
            x1, y1, x2, y2 = bbox.astype(int)
            
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
