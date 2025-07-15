#!/usr/bin/env python3

"""
Objects Detection with yolo on webcam
"""




# Detecting Objects on Image with OpenCV deep learning library


# Reading RGB image

# Loading Yolo v3 Network

##Inferencing the image

# Getting Bounding Boxes

# NMR -Non Max suppression

# Drawing Bounding Boxes with Labels


# Importing needed libraries
import numpy as np
import cv2
import time

"""
Start of:
Reading stream video from camera
"""

# Defining 'VideoCapture' object
# and reading stream video from camera

# camera = cv2.VideoCapture('Carla_RGB_SEMANTIC.mp4')

# Preparing variables for spatial dimensions of the frames
def camera(frame):
    # Loading COCO class labels from file
    # Opening file
    # Pay attention! If you're using Windows, yours path might looks like:
    # r'yolo-coco-data\coco.names'
    # or:
    # 'yolo-coco-data\\coco.names'
    h, w = None, None
    with open(r'C:\Users\ASUS\Desktop\PythonAPI\ControllerTest\detection\detector\obj.names') as f:
        # Getting labels reading every line
        # and putting them into the list
        labels = [line.strip() for line in f]

    # loading config file and weights
    network = cv2.dnn.readNetFromDarknet(r'C:\Users\ASUS\Desktop\PythonAPI\ControllerTest\detection\detector\yolov4-tiny-custom.cfg',
                                        r'C:\Users\ASUS\Desktop\PythonAPI\ControllerTest\detection\detector\yolov4-tiny-custom_last.weights')

    # Getting list with names of all layers from YOLO v3 network
    layers_names_all = network.getLayerNames()

    # Getting only output layers' names that we need from YOLO v3 algorithm
    # with function that returns indexes of layers with unconnected outputs
    layers_names_output = [layers_names_all[i - 1] for i in network.getUnconnectedOutLayers()]
    # Setting minimum probability to eliminate weak predictions
    probability_minimum = 0.5

    # Setting threshold for filtering weak bounding boxes
    # with non-maximum suppression
    threshold = 0.3

    # Generating colours for representing every detected object
    # with function randint(low, high=None, size=None, dtype='l')
    colours = np.random.randint(0, 255, size=(len(labels), 3), dtype='uint8')
    # milimetre cinsinden değerleri girelim.


    focalLength = 200
    tabela_uzunlugu = 500
    bilinen_uzaklik = 784


    # def lidarCallback(x):
    #     print("lidar",x.lidar)


    # rospy.Subscriber("/lidar", Lidar, lidarCallback)


    def focal_lenght_calculator(bilinen_uzaklik, box_height, tabela_uzunlugu):
        global focalLength
        focalLength = (bilinen_uzaklik * box_height) / tabela_uzunlugu


    def distance_to_camera(tabela_uzunlugu, focalLength, pixel_uzunlugu):
        return (tabela_uzunlugu * focalLength) / pixel_uzunlugu


    # Defining loop for catching frames
    # Capturing frame-by-frame from camera           
    #  

    # Getting spatial dimensions of the frame
    # we do it only once from the very beginning
    # all other frames have the same dimension
    if w is None or h is None:
        # Slicing from tuple only first two elements
        h, w = frame.shape[:2]

    # Getting blob from current frame
    # The 'cv2.dnn.blobFromImage' function returns 4-dimensional blob from current
    # frame after mean subtraction, normalizing, and RB channels swapping
    # Resulted shape has number of frames, number of channels, width and height
    # E.G.:
    # blob = cv2.dnn.blobFromImage(image, scalefactor=1.0, size, mean, swapRB=True)
    # Preprocess the frame
# BGR formatına dönüştür
    h, w, _ = frame.shape

    # Giriş görüntüsünün derinliğini kontrol et
    if frame.dtype == np.float64:
        frame = frame.astype(np.float32)

    # BGR formatına dönüştür
    bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Giriş görüntüsünü blob formatına çevir
    blob = cv2.dnn.blobFromImage(bgr_frame, 1 / 255.0, (416, 416), swapRB=True, crop=False)

    network.setInput(blob)
        
    start = time.time()
    output_from_network = network.forward(layers_names_output)
    end = time.time()

    # Showing spent time for single current frame
    # print('Current frame took {:.5f} seconds'.format(end - start))

    # Preparing lists for detected bounding boxes,
    # obtained confidences and class's number
    bounding_boxes = []
    confidences = []
    class_numbers = []

    # Going through all output layers after feed forward pass
    for result in output_from_network:
        # Going through all detections from current output layer
        for detected_objects in result:
            # Getting 80 classes' probabilities for current detected object
            scores = detected_objects[5:]
            # Getting index of the class with the maximum value of probability
            class_current = np.argmax(scores)
            # Getting value of probability for defined class
            confidence_current = scores[class_current]

            if confidence_current > probability_minimum:

                box_current = detected_objects[0:4] * np.array([w, h, w, h])

                x_center, y_center, box_width, box_height = box_current
                x_min = int(x_center - (box_width / 2))
                y_min = int(y_center - (box_height / 2))

                # Adding results into prepared lists
                bounding_boxes.append([x_min, y_min,
                                    int(box_width), int(box_height)])
                confidences.append(float(confidence_current))
                class_numbers.append(class_current)

        results = cv2.dnn.NMSBoxes(bounding_boxes, confidences,
                                probability_minimum, threshold)

        if len(results) > 0:
            for i in results.flatten():

                x_min, y_min = bounding_boxes[i][0], bounding_boxes[i][1]
                box_width, box_height = bounding_boxes[i][2], bounding_boxes[i][3]


                text_box_current = '{}: {:.4f}'.format(labels[int(class_numbers[i])],
                                                    confidences[i])
                print(text_box_current)


                print(labels[int(class_numbers[i])],confidences[i])

        return frame
