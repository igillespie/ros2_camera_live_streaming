import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import subprocess
import os
import threading


class CameraLiveStreamer(Node):
    def __init__(self):
        super().__init__('camera_live_streamer')

        # Declare parameters
        self.declare_parameter('input_topic', '/camera_frames/raw')
        self.declare_parameter('hls_directory', '/var/www/html/hls')
        self.declare_parameter('hls_playlist_name', 'live.m3u8')
        self.declare_parameter('segment_time', 1.0)  # Segment duration in seconds
        self.declare_parameter('width', 640)  # Default width
        self.declare_parameter('height', 480)  # Default height
        self.declare_parameter('fps', 30)  # Default frames per second

        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.hls_directory = self.get_parameter('hls_directory').value
        self.hls_playlist_name = self.get_parameter('hls_playlist_name').value
        self.segment_time = self.get_parameter('segment_time').get_parameter_value().double_value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value

        # Ensure HLS directory exists
        os.makedirs(self.hls_directory, exist_ok=True)

        # FFmpeg process
        self.ffmpeg_process = None
        self.stop_event = threading.Event()

        # Subscriber
        self.subscription = self.create_subscription(
            Image, self.input_topic, self.process_frame, 10)

        self.get_logger().info(
            f"CameraLiveStreamer node initialized. Listening on topic: {self.input_topic}"
        )

    def start_ffmpeg(self):
        """Start FFmpeg for HLS streaming."""
        hls_output_path = os.path.join(self.hls_directory, self.hls_playlist_name)
        ffmpeg_cmd = [
            'ffmpeg',
            '-y',  # Overwrite existing files
            '-f', 'rawvideo',
            '-pixel_format', 'yuv420p',
            '-video_size', f"{self.width}x{self.height}",
            '-framerate', str(self.fps),
            '-i', '-',  # Input from stdin
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-f', 'hls',
            '-hls_time', str(self.segment_time),
            '-hls_list_size', '3',  # Keep only the last 3 segments
            '-hls_flags', 'append_list+delete_segments+split_by_time',
            hls_output_path
        ]
        self.ffmpeg_process = subprocess.Popen(
            ffmpeg_cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)

        self.get_logger().info("FFmpeg process started for HLS streaming.")

    def process_frame(self, msg: Image):
        """Callback for processing incoming frames."""
        try:
            if not self.ffmpeg_process or self.ffmpeg_process.stdin.closed:
                self.get_logger().info(
                    f"Starting FFmpeg process with resolution {self.width}x{self.height} "
                    f"at {self.fps} FPS..."
                )
                self.clear_hls_directory()
                self.start_ffmpeg()


            # Write raw image data to FFmpeg's stdin
            self.ffmpeg_process.stdin.write(bytearray(msg.data))
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

    def clear_hls_directory(self):
        """Remove all files in the HLS directory, with safeguards."""
        try:
            # Ensure the directory is in a safe path
            if not (self.hls_directory.startswith('/home') or self.hls_directory.startswith('/var/www')):
                self.get_logger().error(
                    f"Refusing to clear directory outside of /home or /var/www: {self.hls_directory}"
                )
                return

            # List and delete files
            for filename in os.listdir(self.hls_directory):
                file_path = os.path.join(self.hls_directory, filename)
                if os.path.isfile(file_path):
                    os.unlink(file_path)  # Remove the file
                    self.get_logger().info(f"Deleted file: {file_path}")
                elif os.path.isdir(file_path):
                    os.rmdir(file_path)  # Remove the directory if empty
        except Exception as e:
            self.get_logger().error(f"Error clearing HLS directory: {e}")

    def destroy_node(self):
        """Shut down the FFmpeg process."""
        self.stop_event.set()
        if self.ffmpeg_process:
            self.ffmpeg_process.stdin.close()
            self.ffmpeg_process.terminate()
            self.ffmpeg_process.wait()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraLiveStreamer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()