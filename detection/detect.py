#!/usr/bin/env python3

"""
Objects Detection with YOLO on webcam
"""

import numpy as np
import cv2
import time


class YOLODetector:
    def __init__(self):
        labels_path = 'detection/detector/obj.names'
        config_path = 'detection/detector/yolov4-tiny-custom.cfg'
        weights_path = 'detection/detector/yolov4-tiny-custom_last.weights'
        # Loading COCO class labels from file
        with open(labels_path) as f:
            self.labels = [line.strip() for line in f]

        # Load YOLO v3 Network
        self.network = cv2.dnn.readNetFromDarknet(config_path, weights_path)

        # Get the names of all layers from YOLO network
        layers_names_all = self.network.getLayerNames()

        # Get output layers' names that we need from YOLO algorithm
        self.layers_names_output = [layers_names_all[i - 1] for i in self.network.getUnconnectedOutLayers()]

        # Setting minimum probability to eliminate weak predictions
        self.probability_minimum = 0.5

        # Setting threshold for filtering weak bounding boxes with non-maximum suppression
        self.threshold = 0.3

        # Generating colors for representing every detected object
        self.colors = np.random.randint(0, 255, size=(len(self.labels), 3), dtype='uint8')

        # Initialize frame dimensions
        self.h, self.w = None, None

    def process_frame(self, frame):
        # Ensure the frame is a valid image
        if frame is None:
            raise ValueError("Frame is None")

        # Check the type and shape of the frame
        #print(f"Frame type: {type(frame)}, Frame shape: {frame.shape}, Frame dtype: {frame.dtype}")

        # Ensure the frame is in the correct format (8-bit, 3-channel)
        if frame.dtype != np.uint8:
            print("Converting frame to uint8")
            frame = frame.astype(np.uint8)
        if len(frame.shape) != 3 or frame.shape[2] != 3:
            raise ValueError("Frame is not in the correct format: Expected 3 channels (BGR)")
        
        # Keep original resolution for display
        original_frame = frame.copy()
        
        # Get spatial dimensions of the frame
        if self.w is None or self.h is None:
            self.h, self.w = frame.shape[:2]

        # Get blob from current frame
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416), swapRB=True, crop=False)

        # Forward pass with our blob and only through output layers
        self.network.setInput(blob)
        output_from_network = self.network.forward(self.layers_names_output)

        bounding_boxes = []
        confidences = []
        class_numbers = []

        for result in output_from_network:
            for detected_objects in result:
                scores = detected_objects[5:]
                class_current = np.argmax(scores)
                confidence_current = scores[class_current]

                if confidence_current > self.probability_minimum:
                    box_current = detected_objects[0:4] * np.array([self.w, self.h, self.w, self.h])
                    x_center, y_center, box_width, box_height = box_current
                    x_min = int(x_center - (box_width / 2))
                    y_min = int(y_center - (box_height / 2))

                    bounding_boxes.append([x_min, y_min, int(box_width), int(box_height)])
                    confidences.append(float(confidence_current))
                    class_numbers.append(class_current)

        results = cv2.dnn.NMSBoxes(bounding_boxes, confidences, self.probability_minimum, self.threshold)

        # Track cone detection for obstacle system
        cone_detected = False
        
        if len(results) > 0:
            for i in results.flatten():
                x_min, y_min = bounding_boxes[i][0], bounding_boxes[i][1]
                box_width, box_height = bounding_boxes[i][2], bounding_boxes[i][3]
                colour_box_current = self.colors[class_numbers[i]].tolist()
                text_box_current = f'{self.labels[int(class_numbers[i])]}: {confidences[i]:.4f}'

                # Check bounding box values before drawing
                #print(f"Drawing box: x_min={x_min}, y_min={y_min}, box_width={box_width}, box_height={box_height}")

                cv2.rectangle(original_frame, (x_min, y_min), (x_min + box_width, y_min + box_height), colour_box_current, 2)
                cv2.putText(original_frame, text_box_current, (x_min, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour_box_current, 2)

                #print(self.labels[int(class_numbers[i])], confidences[i])
                
                # Store the detected label
                detected_label = self.labels[int(class_numbers[i])]
                
                # Check for cone detection
                detected_class = self.labels[int(class_numbers[i])]
                if detected_class == "cone":
                    cone_detected = True
                    
                    # Calculate cone center and distance (similar to color detection)
                    center_x = x_min + box_width // 2
                    center_y = y_min + box_height
                    
                    # Simple distance estimation based on bounding box height
                    estimated_distance = max(1.0, min(15.0, (50.0 / max(box_height, 10)) * 10))
                    
                    # Calculate relative position
                    horizontal_offset = (center_x - self.w // 2) / self.w * 2.0
                    cone_x = estimated_distance * horizontal_offset
                    cone_y = estimated_distance
                    
                    # Update obstacle data (similar to cone_detector)
                    from Util import Global
                    cone_width = 0.3
                    left_point = [cone_y, cone_x - cone_width/2, 0.3]
                    right_point = [cone_y, cone_x + cone_width/2, 0.3]
                    Global.dubaEngel = (left_point, right_point)
                    Global.dubaVar = True
                    
                    print(f"YOLO detected cone at: forward={cone_y:.2f}m, side={cone_x:.2f}m")
                    break

        # Clear obstacle data if no cones detected by YOLO
        if not cone_detected:
            from Util import Global
            if hasattr(Global, 'dubaVar') and Global.dubaVar:
                Global.dubaEngel = ()
                Global.dubaVar = False

        # If we found any objects, return the label of the first one (highest confidence)
        detected_label = detected_label if 'detected_label' in locals() else "None"
        
        # Return the original high-quality frame with boxes drawn on it
        return original_frame, detected_label