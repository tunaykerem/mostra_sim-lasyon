import cv2
import numpy as np
from ultralytics import YOLO
import supervision as sv
import time

class TrafficSignDetector:
    def __init__(self, model_path, left_turn_template_path, right_turn_template_path):
        self.model = YOLO(model_path)
        self.no_left_turn_template = cv2.imread(left_turn_template_path, 0)
        self.no_right_turn_template = cv2.imread(right_turn_template_path, 0)
        self.box_annotator = sv.BoxAnnotator(thickness=2, text_thickness=1, text_scale=0.5)

    def preprocess_image(self, image):
        start_time = time.time()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        gray = cv2.equalizeHist(gray)
        edges = cv2.Canny(gray, 50, 150)
        end_time = time.time()
        print(f"preprocess_image took {end_time - start_time:.4f} seconds")
        return edges

    def match_template(self, roi, template):
        start_time = time.time()
        res = cv2.matchTemplate(roi, template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, _ = cv2.minMaxLoc(res)
        end_time = time.time()
        print(f"match_template took {end_time - start_time:.4f} seconds")
        return max_val

    def process_frame(self, img):
        start_time = time.time()
        turn_labels = []

        results = self.model(img)  # Modeli burada çağırıyoruz
        
        for result in results:
            if hasattr(result, 'boxes'):
                detections = sv.Detections.from_yolov8(result)

                for detection_idx in range(len(detections)):
                    class_id = detections.class_id[detection_idx]
                    class_name = self.model.model.names.get(class_id, "Unknown")

                    if class_id == 2:
                        x1, y1, x2, y2 = detections.xyxy[detection_idx].astype(int)
                        cropped_frame = img[y1:y2, x1:x2]

                        preprocessed_cropped_frame = self.preprocess_image(cropped_frame)
                        preprocessed_cropped_frame_resized = cv2.resize(preprocessed_cropped_frame, (100, 100))

                        left_score = self.match_template(preprocessed_cropped_frame_resized, self.no_left_turn_template)
                        right_score = self.match_template(preprocessed_cropped_frame_resized, self.no_right_turn_template)

                        if left_score > right_score:
                            turn_label = "sola_"
                        else:
                            turn_label = "saga_"
                    else:
                        turn_label = ""

                    turn_labels.append(turn_label)

                labels = [
                    f"{tracker_id} {turn_labels[idx]+class_name} {confidence:0.2f}"
                    for idx, (_, confidence, class_id, tracker_id) in enumerate(detections)
                ]

                if isinstance(img, np.ndarray):
                    # Ensure the image is in the correct format
                    if img.dtype != np.uint8:
                        img = img.astype(np.uint8)
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # Changed from COLOR_RGB2BGR to COLOR_BGR2RGB

                img = self.box_annotator.annotate(scene=img, detections=detections, labels=labels)

        end_time = time.time()
        print(f"process_frame took {end_time - start_time:.4f} seconds")

        return img, labels

