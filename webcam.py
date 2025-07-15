import cv2
from ultralytics import YOLO
from ultralytics import YOLO
import supervision as sv
import torch

def image_process(img,model):

    # İşlem süresini ölçmek için başlangıç zamanı
    import time
    start_time = time.time()

    # Track fonksiyonu ile sonuçları al 
    for result in model.track(img, show=False, stream=True, agnostic_nms=True, verbose=True, persist=True):
        
        frame = result.orig_img
        detections = sv.Detections.from_yolov8(result)
        for i in range(len(detections.xyxy)):
            x1, y1, x2, y2 = detections.xyxy[i]
            label = detections.class_names[detections.cls[i]]
            confidence = detections.conf[i]

            # Dikdörtgen çiz
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

            # Etiket ve güven skorunu çiz
            cv2.putText(frame, f"{label} {confidence:.2f}", (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # İşlem süresini hesapla
    elapsed_time = time.time() - start_time
    print(f"Toplam İşlem Süresi: {elapsed_time:.2f} saniye")

    return frame
