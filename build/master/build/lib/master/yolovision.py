from ultralytics import YOLO
import cv2
import numpy as np

class YoloPimp():
    def __init__(self):
        self.model = YOLO('yolov8n-seg.pt')

    def predict(self, frame):
        results = self.model.predict(frame, classes=[0], stream=False, verbose=False)
        return results
