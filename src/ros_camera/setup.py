from setuptools import setup, find_packages

package_name = 'ros_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_stack_launch.py']),
        ('share/' + package_name + '/config', ['config/camera_params.yaml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS 2 package for camera-related nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'raw_camera_publisher = nodes.raw_camera_publisher:main',
            'camera_live_streamer = nodes.camera_live_streamer:main',
            'compressed_image_publisher = nodes.compressed_image_publisher:main',
        ],
    },
)