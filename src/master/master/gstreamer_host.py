from threading import Thread
import gi
import time

import cairo

gi.require_version("Gst", "1.0")
gi.require_version("GstVideo", "1.0")
from gi.repository import Gst, GLib, GstVideo

class Gstream():
    def __init__(self):
        self.hosting_ip = "10.0.3.244"

        self.user_data = {"video_info": None}
        self.width = 0
        self.height = 0

        self.text_display_start_time = 0
        self.is_text_displaying = False
        self.text_display_duration = 3

        self.speed_text = 0
        self.stanley_running_text = "Not running"
        self.current_gear_text = ""
        self.speed_limiter_text = 100
        self.gps_status_text = ""
        self.stanley_at_final_point_text = "Not complete"
        self.stanley_k_text = 0
        self.stanley_v_text = 0
        self.stanley_path_reset_state = 0

    def text_overlay(self, cairo_ctx, text, x_position, y_position, font_size = 20, r = 1, g = 1, b = 1, a = 1):
        cairo_ctx.select_font_face("Sans", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
        cairo_ctx.set_font_size(font_size)

        cairo_ctx.save()

        cairo_ctx.set_source_rgba(0, 0, 0, 1)
        cairo_ctx.set_line_width(2)
        cairo_ctx.move_to(x_position, y_position)
        cairo_ctx.text_path(text)
        cairo_ctx.stroke()
        cairo_ctx.restore()

        cairo_ctx.set_source_rgba(r, g, b, a)
        cairo_ctx.move_to(x_position, y_position)
        cairo_ctx.show_text(text) 

    def draw_overlay(self, cairo_ctx):
        image_surface = cairo.ImageSurface.create_from_png("/home/pimpmobile/ros2_ws/src/master/icons/pimpmyride.png")

        image_width = 100
        image_height = 100

        scale_x = image_width / image_surface.get_width()
        scale_y = image_height / image_surface.get_height()

        cairo_ctx.save()

        x_position = self.width - image_width # left-right is 0 - 1280 
        y_position = 0 # up-down is 0 - 720

        cairo_ctx.translate(x_position, y_position)
        cairo_ctx.scale(scale_x, scale_y)

        cairo_ctx.set_source_surface(image_surface, 0, 0)
        cairo_ctx.paint()
        cairo_ctx.restore()

        ### TEXT ###

            #### BOTTOM RIGHT ####

        #CURRENT GEAR
        self.text_overlay(cairo_ctx, f"Gear: {self.current_gear_text}", self.width - 130, self.height - 20)

        #SPEED LIMITER
        self.text_overlay(cairo_ctx, f"Speed Limiter: {self.speed_limiter_text}%", self.width - 220, self.height - 40)

            #### BOTTOM LEFT ####

        #SPEED
        self.text_overlay(cairo_ctx, f"Speed: {self.speed_text} m/s", 0, self.height - 20)

        #AUTONOMOUS DRIVE STATUS
        if self.stanley_running_text == "Running":
            self.text_overlay(cairo_ctx, f"Autonomous drive: {self.stanley_running_text}", 0, self.height - 40, g = 0, b = 0)
        else:
            self.text_overlay(cairo_ctx, f"Autonomous drive: {self.stanley_running_text}", 0, self.height - 40)

        #AUTONOMOUS PATH STATUS
        self.text_overlay(cairo_ctx, f"Path status: {self.stanley_at_final_point_text}", 0, self.height - 60)

        #STANLEY V
        self.text_overlay(cairo_ctx, f"Stanley V: {self.stanley_v_text}", 0, self.height - 80)

        #STANLEY K
        self.text_overlay(cairo_ctx, f"Stanley K: {self.stanley_k_text}", 0, self.height - 100)

            #### TOP LEFT ####

        #GPS STATUS/COORDINATES
        self.text_overlay(cairo_ctx, f"GPS Status: {self.gps_status_text}", 0, 25)

            #### MIDDLE ####

        if self.stanley_path_reset_state == 1:
            self.text_overlay(cairo_ctx, "Resetting path yaw & index...", self.width / 2 - 220, self.height / 2, 40, 1, 0, 0, 1)
        
        #TODO: MINIMAP


    def on_draw_callback(self, overlay, cairo_ctx, timestamp, duration, user_data):
        # Get the video frame information (width, height, etc.)
        video_info = self.user_data["video_info"]

        if video_info is None:
            return
        
        self.width = video_info.width
        self.height = video_info.height
        
        self.draw_overlay(cairo_ctx)

    def on_caps_callback(self, overlay, caps, user_data):
        video_info = GstVideo.VideoInfo()
        video_info.from_caps(caps)
        
        self.user_data["video_info"] = video_info

    def start_stream(self):
        Gst.init("")

        self.main_loop = GLib.MainLoop()
        thread = Thread(target=self.main_loop.run)
        thread.start()

        self.pipeline = Gst.parse_launch(
            "v4l2src device=/dev/video0 ! "
            "image/jpeg,width=1280,height=720 ! "
            "jpegdec ! "
            "videoconvert ! "
            "cairooverlay name=overlay ! "
            "videoconvert ! "
            "x264enc tune=zerolatency bitrate=1500 speed-preset=superfast ! "
            "rtph264pay ! "
            f"udpsink host={self.hosting_ip} port=5000 sync=false"
        )

        self.overlay = self.pipeline.get_by_name("overlay")
        
        self.overlay.connect("draw", self.on_draw_callback, self.user_data)
        self.overlay.connect("caps-changed", self.on_caps_callback, self.user_data)

        self.pipeline.set_state(Gst.State.PLAYING)


if __name__ == "__main__":
    gstream_obj = Gstream()
    gstream_obj.start_stream()
    while True:
        sleep(0.1)