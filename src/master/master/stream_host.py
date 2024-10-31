import cv2
import numpy as np
import cairo
from master.cairo_overlay import CairoOverlay
from master.yolovision import YoloPimp

# Initialize Gstream class for overlay
cairo_overlay = CairoOverlay()

yolo_obj = YoloPimp()
source = 0
fps = 30.0

hosting_ip = "10.0.6.239"
port = 5007

width = 1280
height = 720

cap = cv2.VideoCapture(source)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, fps)

gst_str = (
    f"appsrc ! video/x-raw,format=BGR,width={width},height={height},framerate={int(fps)}/1 ! "
    f"videoconvert ! x264enc tune=zerolatency bitrate=1500 speed-preset=superfast ! "
    f"rtph264pay ! udpsink host={hosting_ip} port={port}"
)
out = cv2.VideoWriter(gst_str, cv2.CAP_GSTREAMER, 0, fps, (width, height), True)

if not out.isOpened():
    print("Error: Could not open GStreamer pipeline.")
    cap.release()
    exit()

try:
    # Main loop
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to read frame from camera.")
            break

        try:
            #results = model.predict(frame, classes=[0], stream=False, verbose=False)
            results = yolo_obj.predict(frame)
        except Exception as e:
            print(f"Error during YOLO prediction: {e}")
            continue

        annotated_frame = frame.copy()
        for result in results:
            boxes = result.boxes
            masks = result.masks

            if masks is not None:
                annotated_frame = result.plot()

            for box in boxes:
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                x_min, y_min, x_max, y_max = xyxy

                x_center = int((x_min + x_max) / 2)
                y_center = int((y_min + y_max) / 2)

                position = ((x_center - (width / 2)) / (width / 2)) * 100
                position = int(position) 
                position_text = f"Offset: {position}"
                cv2.putText(annotated_frame, position_text, (x_min, y_min - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.circle(annotated_frame, (x_center, y_center), 5, (0, 255, 0), -1)

        # Create Cairo overlay for each frame
        overlay = np.zeros((height,width, 4), dtype=np.uint8)  # Create RGBA overlay
        surface = cairo.ImageSurface.create_for_data(overlay, cairo.FORMAT_ARGB32, width, height)
        ctx = cairo.Context(surface)

        # Draw overlay with Gstream's method
        cairo_overlay.draw_overlay(ctx)

        # Convert the Cairo overlay to BGR and blend it with the frame
        overlay_bgr = cv2.cvtColor(overlay, cv2.COLOR_BGRA2BGR)
        annotated_frame = cv2.addWeighted(annotated_frame, 1.0, overlay_bgr, 1, 0)

        # Write to output
        out.write(annotated_frame)

finally:
    cap.release()
    out.release()