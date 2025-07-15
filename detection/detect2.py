#!/usr/bin/env python3

"""
Objects Detection with YOLO on webcam
"""

import numpy as np
import cv2
import time

class YOLODetector:
    def __init__(self):
        labels_path = 'C:/Users/ASUS/Desktop/PythonAPI/son/detection/detector/obj.names'
        config_path = 'C:/Users/ASUS/Desktop/PythonAPI/son/detection/detector/yolov4-tiny-custom.cfg'
        weights_path = 'C:/Users/ASUS/Desktop/PythonAPI/son/detection/detector/yolov4-tiny-custom_last.weights'
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
     
    def process_frame(self,frame):
        if self.w is None or self.h is None:
        # Slicing from tuple only first two elements
            self.h, self.w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416), swapRB=True, crop=False)

        
        self.network.setInput(blob)  # setting blob as input to the network
        start = time.time()
        output_from_network = self.network.forward(self.layers_names_output)
        end = time.time()
        bounding_boxes = []
        confidences = []
        class_numbers = []
        for result in output_from_network:
            # Going through all detections from current output layer
            for detected_objects in result:
                # Getting 80 classes' probabilities for current detected object
                scores = detected_objects[5:]
                # Getting index of the class with the maximum value of probability
                class_current = np.argmax(scores)
                # Getting value of probability for defined class
                confidence_current = scores[class_current]

                # # Check point
                # # Every 'detected_objects' numpy array has first 4 numbers with
                # # bounding box coordinates and rest 80 with probabilities
                # # for every class
                # print(detected_objects.shape)  # (85,)

                # Eliminating weak predictions with minimum probability
                if confidence_current > self.probability_minimum:
                    # Scaling bounding box coordinates to the initial frame size
                    # YOLO data format keeps coordinates for center of bounding box
                    # and its current width and height
                    # That is why we can just multiply them elementwise
                    # to the width and height
                    # of the original frame and in this way get coordinates for center
                    # of bounding box, its width and height for original frame
                    box_current = detected_objects[0:4] * np.array([w, h, w, h])

                    # Now, from YOLO data format, we can get top left corner coordinates
                    # that are x_min and y_min
                    x_center, y_center, box_width, box_height = box_current
                    x_min = int(x_center - (box_width / 2))
                    y_min = int(y_center - (box_height / 2))

                    # Adding results into prepared lists
                    bounding_boxes.append([x_min, y_min,
                                        int(box_width), int(box_height)])
                    confidences.append(float(confidence_current))
                    class_numbers.append(class_current)
        results = cv2.dnn.NMSBoxes(bounding_boxes, confidences,
                                self.probability_minimum, self.threshold)

        # Checking if there is at least one detected object
        # after non-maximum suppression
        if len(results) > 0:
            # Going through indexes of results
            for i in results.flatten():
                # Getting current bounding box coordinates,
                # its width and height
                x_min, y_min = bounding_boxes[i][0], bounding_boxes[i][1]
                box_width, box_height = bounding_boxes[i][2], bounding_boxes[i][3]

                # Preparing colour for current bounding box
                # and converting from numpy array to list
                colour_box_current = self.colours[class_numbers[i]].tolist()

                # # # Check point
                # print(type(colour_box_current))  # <class 'list'>
                # print(colour_box_current)  # [172 , 10, 127]

                # Drawing bounding box on the original current frame
                cv2.rectangle(frame, (x_min, y_min),
                            (x_min + box_width, y_min + box_height),
                            colour_box_current, 2)

                # Preparing text with label and confidence for current bounding box
                text_box_current = '{}: {:.4f}'.format(self.labels[int(class_numbers[i])],
                                                    confidences[i])

                # Putting text with label and confidence on the original image
                cv2.putText(frame, text_box_current, (x_min, y_min - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour_box_current, 2)
        return frame