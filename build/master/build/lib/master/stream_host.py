import cv2
import numpy as np
import cairo
import threading
from master.cairo_overlay import CairoOverlay
from master.yolovision import YoloPimp

class Gstream():
    def __init__(self):
        self.cairo_overlay_obj = CairoOverlay()
        self.yolo_obj = YoloPimp()

        source = 0
        hosting_ip = "10.0.6.239"
        port = 5007
        fps = 30.0
        self.width = 1280
        self.height = 720

        self.annotated_frame = None

        self.cap = cv2.VideoCapture(source)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        #TODO: mjpeg format, ultrafast jne jne jne
        gst_str = (
            f"appsrc ! video/x-raw,format=BGR,self.width={self.width},self.height={self.height},framerate={int(fps)}/1 ! "
            f"videoconvert ! x264enc tune=zerolatency bitrate=1500 speed-preset=superfast ! "
            f"rtph264pay ! udpsink host={hosting_ip} port={port}"
        )
        self.cvgstream_output = cv2.VideoWriter(gst_str, cv2.CAP_GSTREAMER, 0, fps, (self.width, self.height), True)

    def get_cairo_overlay_object(self):
        return self.cairo_overlay_obj

    def draw_yolo_overlay(self, results):
        for result in results:
            boxes = result.boxes
            masks = result.masks

            if masks is not None:
                self.annotated_frame = result.plot()

            for box in boxes:
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                x_min, y_min, x_max, y_max = xyxy

                x_center = int((x_min + x_max) / 2)
                y_center = int((y_min + y_max) / 2)

                position = ((x_center - (self.width / 2)) / (self.width / 2)) * 100
                position = int(position) 
                position_text = f"Offset: {position}"
                print(position_text)
                cv2.putText(self.annotated_frame, position_text, (x_min, y_min - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.circle(self.annotated_frame, (x_center, y_center), 5, (0, 255, 0), -1)

    def start_stream(self):
        if not self.cvgstream_output.isOpened():
            print("Error: Could not open GStreamer pipeline.")
            self.cap.release()
            exit()
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Error: Failed to read frame from camera.")
                    break

                try:
                    results = self.yolo_obj.predict(frame)
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
                self.annotated_frame = cv2.addWeighted(self.annotated_frame, 1.0, overlay_bgr, 1, 0)

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