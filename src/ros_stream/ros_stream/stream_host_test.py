import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2


class RosStream(Node):
    def __init__(self):
        super().__init__('video_stream')
        self.publisher = self.create_publisher(CompressedImage, 'ros_stream_pimp', 10)
        self.timer_period = 0.033  # Approximately 30 FPS
        self.timer = self.create_timer(self.timer_period, self.stream_callback)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open the video capture device.")
            self.cap.release()
            raise RuntimeError("Could not open video device.")

    def stream_callback(self):
        ret, frame = self.cap.read()

        if ret:
            # Compress frame to JPEG format
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, compressed_frame = cv2.imencode('.jpg', gray_frame, [cv2.IMWRITE_JPEG_QUALITY, 30])

            # Create CompressedImage message
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = compressed_frame.tobytes()
            self.publisher.publish(msg)
            self.get_logger().info("Published frame.")

        else:
            self.get_logger().warning("Failed to read frame from video capture.")

    def destroy_node(self):
        self.cap.release()  # Release video capture device
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    video_pub = RosStream()
    try:
        rclpy.spin(video_pub)
    except KeyboardInterrupt:
        video_pub.get_logger().info("Shutting down video stream node.")
    finally:
        video_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()