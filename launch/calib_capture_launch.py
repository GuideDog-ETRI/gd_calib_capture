import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/zed/zed_node/left_raw/image_raw_color',
            description='Topic name for the camera'
        ),
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/lidar_topic',
            description='Topic name for the LiDAR'
        ),
        DeclareLaunchArgument(
            'image_file_prefix',
            default_value='image',
            description='Prefix for saved image files'
        ),
        DeclareLaunchArgument(
            'pcd_file_prefix',
            default_value='pointcloud',
            description='Prefix for saved point cloud files'
        ),
        Node(
            package='gd_calib_capture',
            executable='calib_capture_node',
            name='calib_capture_node',
            output='screen',
            parameters=[{
                'camera_topic': LaunchConfiguration('camera_topic'),
                'lidar_topic': LaunchConfiguration('lidar_topic'),
                'image_file_prefix': LaunchConfiguration('image_file_prefix'),
                'pcd_file_prefix': LaunchConfiguration('pcd_file_prefix'),
            }]
        ),
    ])