from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_camera',
            executable='raw_camera_publisher',
            name='raw_camera_publisher',
            parameters=[
                {'width': 1280, 'height': 720, 'fps': 30, 'codec': 'yuv420'}
            ]
        ),
        Node(
            package='ros_camera',
            executable='camera_live_streamer',
            name='camera_live_streamer',
            parameters=[
                {'input_topic': '/camera_frames/raw',
                 'hls_directory': '/tmp/hls',
                 'hls_playlist_name': 'live.m3u8',
                 'segment_time': 1.0}
            ]
        )
    ])