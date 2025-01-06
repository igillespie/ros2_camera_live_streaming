import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np

class CompressedImagePublisher(Node):
    def __init__(self):
        super().__init__('compressed_image_publisher')

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

        self.get_logger().info("Compressed Image Publisher Node initialized.")

    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image message to a YUV420 numpy array
            yuv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height + msg.height // 2, msg.width))

            # Decode YUV420 to BGR (OpenCV format)
            bgr_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_I420)

            # Encode BGR image to JPEG
            success, jpeg_data = cv2.imencode('.jpg', bgr_image, [cv2.IMWRITE_JPEG_QUALITY, 70])
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