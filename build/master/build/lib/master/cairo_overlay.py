import cairo

class CairoOverlay():
    def __init__(self, width=1280, height=720):
        self.user_data = {"video_info": None}
        self.width = width
        self.height = height

        self.speed_text = 0
        self.stanley_running_text = "Not running"
        self.current_gear_text = ""
        self.speed_limiter_text = 100
        self.gps_status_text = ""
        self.stanley_at_final_point_text = "Not complete"
        self.gps_path_saving_text = "Not saving"
        self.stanley_k_text = 0
        self.stanley_v_text = 0
        self.stanley_path_reset_state = 0
        self.pid_values = [0, 0, 0, 0]

    def text_overlay(self, cairo_ctx, text, x_position, y_position, font_size=20, r=1, g=1, b=1, a=1):
        cairo_ctx.select_font_face("Sans", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
        cairo_ctx.set_font_size(font_size)
        cairo_ctx.save()

        # Draw black shadow
        cairo_ctx.set_source_rgba(0, 0, 0, 1)
        cairo_ctx.set_line_width(2)
        cairo_ctx.move_to(x_position, y_position)
        cairo_ctx.text_path(text)
        cairo_ctx.stroke()
        cairo_ctx.restore()

        # Draw actual text
        cairo_ctx.set_source_rgba(r, g, b, a)
        cairo_ctx.move_to(x_position, y_position)
        cairo_ctx.show_text(text)

    def draw_overlay(self, cairo_ctx):
        # Draw the image overlay in the top right corner
        image_surface = cairo.ImageSurface.create_from_png("/home/pimpmobile/ros2_ws/src/master/icons/pimpmyride.png")
        image_width, image_height = 100, 100
        scale_x = image_width / image_surface.get_width()
        scale_y = image_height / image_surface.get_height()

        cairo_ctx.save()
        x_position = self.width - image_width
        y_position = 0
        cairo_ctx.translate(x_position, y_position)
        cairo_ctx.scale(scale_x, scale_y)
        cairo_ctx.set_source_surface(image_surface, 0, 0)
        cairo_ctx.paint()
        cairo_ctx.restore()

        # Draw various text overlays
        # Bottom Right
        self.text_overlay(cairo_ctx, f"Gear: {self.current_gear_text}", self.width - 130, self.height - 20)
        self.text_overlay(cairo_ctx, f"Speed Limiter: {self.speed_limiter_text}%", self.width - 220, self.height - 40)

        # Bottom Left
        self.text_overlay(cairo_ctx, f"Speed: {self.speed_text} m/s", 0, self.height - 20)
        self.text_overlay(cairo_ctx, f"Autonomous drive: {self.stanley_running_text}", 0, self.height - 40, g=0 if self.stanley_running_text == "Running" else 1, b=0 if self.stanley_running_text == "Running" else 1)
        self.text_overlay(cairo_ctx, f"Path saving: {self.gps_path_saving_text}", 0, self.height - 60)
        self.text_overlay(cairo_ctx, f"Path status: {self.stanley_at_final_point_text}", 0, self.height - 80)
        self.text_overlay(cairo_ctx, f"Stanley V: {self.stanley_v_text}", 0, self.height - 100)
        self.text_overlay(cairo_ctx, f"Stanley K: {self.stanley_k_text}", 0, self.height - 120)
        self.text_overlay(cairo_ctx, f"PID: {self.pid_values}", 0, self.height - 140)

        # Top Left
        self.text_overlay(cairo_ctx, f"GPS Status: {self.gps_status_text}", 0, 25)

        # Middle
        if self.stanley_path_reset_state == 1:
            self.text_overlay(cairo_ctx, "Resetting path yaw & index...", self.width / 2 - 220, self.height / 2, 40, 1, 0, 0, 1)