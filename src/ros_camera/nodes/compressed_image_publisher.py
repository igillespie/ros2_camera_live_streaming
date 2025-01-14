import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np

class CompressedImagePublisher(Node):
    def __init__(self):
        super().__init__('compressed_image_publisher')

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 360)
        self.declare_parameter('jpeg_quality', 50)  # Default JPEG quality

        # Retrieve parameters
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        # Subscriber to the raw YUV420 images
        self.subscription = self.create_subscription(
            Image,
            '/camera_frames/raw',  # Topic for raw images
            self.image_callback,
            10
        )

        # Publisher for the JPEG-compressed images
        self.publisher = self.create_publisher(
            CompressedImage,
            '/camera_frames/compressed',  # Topic for JPEG images
            10
        )

        # Validate JPEG quality (should be between 0 and 100)
        if not (0 <= self.jpeg_quality <= 100):
            self.get_logger().warn(
                f"Invalid JPEG quality: {self.jpeg_quality}. Clamping to range 0-100."
            )
            self.jpeg_quality = max(0, min(self.jpeg_quality, 100))

        self.get_logger().info(
            f"Compressed Image Publisher Node initialized with dimensions: {self.width}x{self.height}, "
            f"JPEG quality: {self.jpeg_quality}"
        )



    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image message to a YUV420 numpy array
            yuv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height + msg.height // 2, msg.width))

            # Decode YUV420 to BGR (OpenCV format)
            bgr_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_I420)

            # Resize the BGR image to the target dimensions
            resized_image = cv2.resize(bgr_image, (self.width, self.height), interpolation=cv2.INTER_LINEAR)

            # Encode resized BGR image to JPEG
            success, jpeg_data = cv2.imencode('.jpg', resized_image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if not success:
                self.get_logger().error("Failed to encode image to JPEG format.")
                return

            # Create a CompressedImage ROS message
            jpeg_msg = CompressedImage()
            jpeg_msg.header = msg.header  # Retain the original message's header
            jpeg_msg.format = "jpeg"
            jpeg_msg.data = jpeg_data.tobytes()

            # Publish the JPEG image
            self.publisher.publish(jpeg_msg)
            self.get_logger().debug("Published compressed image.")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()