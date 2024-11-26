import cv2
import numpy as np
import cairo
import threading
import time
from master.cairo_overlay import CairoOverlay
from master.yolovision import YoloPimp

class Gstream():
    def __init__(self):
        self.cairo_overlay_obj = CairoOverlay()
        self.yolo_obj = YoloPimp()

        source = 0
        hosting_ip = "10.0.6.239"
        port = 5007
        fps = 60.0
        self.width = 1280
        self.height = 720

        self.annotated_frame = None

        self.cap = cv2.VideoCapture(source)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        #TODO: mjpeg format, ultrafast jne jne jne
        gst_str = (
            f"appsrc ! queue ! video/x-raw,format=BGR,self.width={self.width},self.height={self.height},framerate={int(fps)}/1 ! "
            f"videoconvert ! queue ! x264enc tune=zerolatency bitrate=2000 speed-preset=veryfast ! "
            f"rtph264pay ! udpsink host={hosting_ip} port={port} sync=false async=false "
        )

        self.cvgstream_output = cv2.VideoWriter(gst_str, cv2.CAP_GSTREAMER, 0, fps, (self.width, self.height), True)

    def get_cairo_overlay_object(self):
        return self.cairo_overlay_obj

    def draw_yolo_overlay(self, results):
        if results is None:
            return

        for result in results:
            # Extract bounding boxes and masks
            boxes = result.boxes  # YOLO detection bounding boxes
            masks = result.masks  # YOLO instance segmentation masks (if any)

            # Draw bounding boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy)
                    class_id = int(box.cls)
                    score = box.conf

                    color = tuple(np.random.randint(0, 255, 3).tolist())
                    label = f"Class {class_id}: {score:.2f}"

                    cv2.rectangle(self.annotated_frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(
                        self.annotated_frame,
                        label,
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        2,
                    )
            if masks is not None:
                for mask in masks:
                    binary_mask = mask.data > 0.5
                    color_mask = np.random.randint(0, 255, (1, 3), dtype=np.uint8)

                    self.annotated_frame[binary_mask] = (
                        self.annotated_frame[binary_mask] * 0.5 + color_mask * 0.5
                    ).astype(np.uint8)


    def start_stream(self):
        if not self.cvgstream_output.isOpened():
            print("Error: Could not open GStreamer pipeline.")
            self.cap.release()
            exit()
        results = None
        try:
            time_end = 0
            while True:
                time_start = time.time()
                ret, frame = self.cap.read()
                if not ret:
                    print("Error: Failed to read frame from camera.")
                    break

                try:
                    if time_start > time_end + 0.5:
                        results = self.yolo_obj.predict(frame)
                        time_end = time.time()
                except Exception as e:
                    print(f"Error during YOLO prediction: {e}")
                    continue

                self.annotated_frame = frame.copy()
                self.draw_yolo_overlay(results)


                # Create Cairo overlay for each frame
                overlay = np.zeros((self.height,self.width, 4), dtype=np.uint8)  # Create RGBA overlay
                surface = cairo.ImageSurface.create_for_data(overlay, cairo.FORMAT_ARGB32, self.width, self.height)
                ctx = cairo.Context(surface)

                # Draw overlay with Gstream's method
                self.cairo_overlay_obj.draw_overlay(ctx)

                # Convert the Cairo overlay to BGR and blend it with the frame
                overlay_bgr = cv2.cvtColor(overlay, cv2.COLOR_BGRA2BGR)
                #overlay_bgr = cv2.cvtColor(overlay, cv2.COLOR_BGR2YUV_I420)

                self.annotated_frame = cv2.addWeighted(self.annotated_frame, 1, overlay_bgr, 1.5, 0)

                self.cvgstream_output.write(self.annotated_frame)

        finally:
            self.cap.release()
            self.cvgstream_output.release()

    def start_stream_in_thread(self):
        stream_thread = threading.Thread(target=self.start_stream)
        stream_thread.daemon = True
        stream_thread.start()

if __name__ == '__main__':
    stream_obj = Gstream()
    stream_obj.start_stream()