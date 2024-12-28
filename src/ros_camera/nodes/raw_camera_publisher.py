import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import subprocess
import threading
import queue


class RawCameraPublisher(Node):
    def __init__(self):
        super().__init__('raw_camera_publisher')

        # Declare parameters
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('codec', 'yuv420')
        self.declare_parameter('topic', '/camera_frames/raw')

        # Retrieve parameters
        self.codec = self.get_parameter('codec').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.topic = self.get_parameter('topic').value

        self.get_logger().info(
            f"Starting video stream with codec={self.codec}, width={self.width}, height={self.height}, fps={self.fps}"
        )

       # Initialize attributes
        self.raw_publisher = self.create_publisher(Image, self.topic, 10)
        self.frame_queue = queue.Queue(maxsize=10)
        self.stop_event = threading.Event()
        self.pipeline = None  # Initialize pipeline to None

        # Start threads
        self.capture_thread = threading.Thread(target=self.start_capture, daemon=True)
        self.capture_thread.start()
        self.error_monitor_thread = threading.Thread(target=self.monitor_errors, daemon=True)
        self.error_monitor_thread.start()

        # Timer for publishing
        self.timer = self.create_timer(1 / self.fps, self.publish_frame)

    def start_capture(self):
        """Captures video frames and stores them in a queue."""
        frame_size = self.width * self.height * 3 // 2
        self.pipeline = subprocess.Popen(
            [
                'libcamera-vid',
                '--codec', self.codec,
                '--width', str(self.width),
                '--height', str(self.height),
                '--framerate', str(self.fps),
                '--nopreview',
                '--timeout', '0',
                '--output', '-'
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=10**6
        )

        while not self.stop_event.is_set():
            try:
                data = self.pipeline.stdout.read(frame_size)
                if len(data) != frame_size:
                    self.get_logger().warning("Incomplete frame received")
                    continue
                self.frame_queue.put(data, block=False)
            except queue.Full:
                self.get_logger().warning("Queue full. Dropping frame.")
            except Exception as e:
                self.get_logger().error(f"Error in capture thread: {e}")

    def monitor_errors(self):
        """Monitors libcamera-vid stderr for errors."""
        while not self.stop_event.is_set() and self.pipeline:
            error_line = self.pipeline.stderr.readline()
            if error_line:
                self.get_logger().error(f"libcamera-vid error: {error_line.decode().strip()}")

    def publish_frame(self):
        """Publishes frames from the queue."""
        if not self.frame_queue.empty():
            try:
                data = self.frame_queue.get_nowait()
                ros_image = Image()
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.height = self.height
                ros_image.width = self.width
                ros_image.encoding = "yuv420"
                ros_image.is_bigendian = False
                ros_image.step = self.width
                ros_image.data = data
                self.raw_publisher.publish(ros_image)
            except Exception as e:
                self.get_logger().error(f"Error publishing frame: {e}")

    def destroy_node(self):
        """Shuts down gracefully."""
        self.stop_event.set()
        if self.pipeline and self.pipeline.poll() is None:
            self.pipeline.terminate()
            self.pipeline.wait()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RawCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()