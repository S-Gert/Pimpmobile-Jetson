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
        #hosting_ip = "10.0.3.244" # Clevon
        hosting_ip = "10.0.6.239" # Clevon Academy
        port = 5007
        fps = 60.0
        self.width = 1280
        self.height = 720

        self.centerx = 0
        self.centery = 0
        self.area_limit_exceeded = False
        
        self.camera_matrix = np.array( [[539.99262974,   0.         , 638.07980163],
                           [  0.,         541.85644195, 328.63733272],
                           [  0.,           0.,           1.        ]])
        self.distortion_coefficients = np.array([[-0.31960382, 0.1087656, -0.00148948, 0.00103482, -0.01740585]])


        self.annotated_frame = None

        self.cap = cv2.VideoCapture(source)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.new_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.distortion_coefficients, (self.width, self.height), 1, (self.width, self.height))
        
        gst_str = (
            f"appsrc ! queue ! video/x-raw,format=BGR,self.width={self.width},self.height={self.height},framerate={int(fps)}/1 ! "
            f"videoconvert ! queue ! x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! "
            f"rtph264pay ! udpsink host={hosting_ip} port={port} sync=false async=false "
        )

        self.cvgstream_output = cv2.VideoWriter(gst_str, cv2.CAP_GSTREAMER, 0, fps, (self.width, self.height), True)

    def get_cairo_overlay_object(self):
        return self.cairo_overlay_obj

    def draw_yolo_overlay(self, results):
        if results is None:
            return

        class_names = ['BUS LANE', 'Yellow Marking', 'Line', 'Crossing', 
                       'Diamond', 'SLOW', 'Arrow Left', 'Arrow Forward',
                       'Arrow Forward - Left', 'Arrow Forward - Right',
                       'Arrow Right', 'Bicycle']

        for result in results:
            boxes = result.boxes
            masks = result.masks

            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy.view(-1).tolist())
                    class_id = int(box.cls)
                    score = box.conf.item()

                    color = (0, 255, 0)
                    label = f"{class_names[class_id]}: {score:.2f}"

                    # cv2.rectangle(self.annotated_frame, (x1, y1), (x2, y2), color, 2)
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
                    binary_mask = mask.data.cpu().numpy() > 0.5
                    binary_mask = np.squeeze(binary_mask)
                    
                    if binary_mask.shape != (self.height, self.width):
                        binary_mask = cv2.resize(binary_mask.astype(np.uint8), 
                                                (self.width, self.height), 
                                                interpolation=cv2.INTER_NEAREST).astype(bool)

                    color_mask = np.array((0, 200, 0), dtype=np.uint8)

                    self.annotated_frame[binary_mask] = (
                        self.annotated_frame[binary_mask] * 0.5 + color_mask * 0.5
                    ).astype(np.uint8)

    def separate_mask(self):
        """
        Separates and processes yellow regions in the bottom half of the frame based on HSV values,
        calculates a single center point between all significant contours, 
        and displays it on the frame along with the contours.
        """
        if self.annotated_frame is None:
            return

        roi_start_y = self.height // 2
        roi_start_x = self.width // 2
        roi_frame = self.annotated_frame[roi_start_y:self.height, :self.width]

        hsv_frame = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)

        lower_hsv = np.array([0, 135, 0])
        upper_hsv = np.array([179, 255, 255])

        hsv_mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)

        kernel = np.ones((5, 5), np.uint8)
        hsv_mask = cv2.morphologyEx(hsv_mask, cv2.MORPH_CLOSE, kernel)
        hsv_mask = cv2.morphologyEx(hsv_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(hsv_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        area_threshold = 7500

        total_x = 0
        total_y = 0
        valid_contour_count = 0

        for contour in contours:
            area = cv2.contourArea(contour)

            if area > area_threshold:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"]) + roi_start_y

                    total_x += center_x
                    total_y += center_y
                    valid_contour_count += 1

                cv2.drawContours(self.annotated_frame, [contour + [0, roi_start_y]], -1, (0, 255, 255), 2)
        

        if valid_contour_count > 0:
            self.area_limit_exceeded = True
            avg_center_x = total_x // valid_contour_count
            avg_center_y = total_y // valid_contour_count

            self.centerx = avg_center_x
            self.centery = avg_center_y
            #print(f"x: {self.centerx}, y: {self.centery}")

            cv2.circle(self.annotated_frame, (avg_center_x, avg_center_y), 8, (0, 0, 255), -1)

            cv2.putText(self.annotated_frame, f"Center: ({avg_center_x}, {avg_center_y})", 
                        (avg_center_x + 10, avg_center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        else:
            self.area_limit_exceeded = False

        #print(self.area_limit_exceeded)


    def start_stream(self):
        if not self.cvgstream_output.isOpened():
            print("Error: Could not open GStreamer pipeline.")
            self.cap.release()
            exit()
        results = None
        try:
            #time_end = 0
            #yolo_predict_interval = 0.2
            while True:
                time_start = time.time()
                ret, frame = self.cap.read()

                if not ret:
                    print("Error: Failed to read frame from camera.")
                    break

                # undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.distortion_coefficients, None, self.new_camera_matrix)
                
                # x, y, w, h = self.roi
                # if self.roi != (0, 0, 0, 0):
                #     undistorted_frame = undistorted_frame[y:y+h, x:x+w]

                # undistorted_frame = cv2.resize(undistorted_frame, (frame.shape[1], frame.shape[0]))
                # frame = undistorted_frame

                # try:
                #     if time_start > time_end + yolo_predict_interval:
                #         results = self.yolo_obj.predict(frame)
                #         time_end = time.time()
                # except Exception as e:
                #     print(f"Error during YOLO prediction: {e}")
                #     continue

                self.annotated_frame = frame.copy()
                #self.draw_yolo_overlay(results)

                self.separate_mask()

                overlay = np.zeros((self.height,self.width, 4), dtype=np.uint8)
                surface = cairo.ImageSurface.create_for_data(overlay, cairo.FORMAT_ARGB32, self.width, self.height)
                ctx = cairo.Context(surface)

                self.cairo_overlay_obj.draw_overlay(ctx)

                overlay_bgr = cv2.cvtColor(overlay, cv2.COLOR_BGRA2BGR)

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