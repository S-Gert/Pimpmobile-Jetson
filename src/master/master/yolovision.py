from ultralytics import YOLO
import cv2
import numpy as np

class YoloPimp():
    def __init__(self):
        self.model = YOLO('/home/pimpmobile/YOLOv8_lane_model/yolov8n_lane_segment/weights/best.pt')

    def predict(self, frame):
        #results = self.model.predict(frame, classes=[0], stream=False, verbose=False)
        results = self.model.predict(frame)
        return results