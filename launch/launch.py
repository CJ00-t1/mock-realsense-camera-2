from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mock_realsense_camera_2',
            executable='mock_realsense_publisher',
            name='mock_realsense_publisher',
            output='screen',
            parameters=[{
                'rgb_folder': 'data/rgb',
                'depth_folder': 'data/depth',
                'camera_info_path': 'config/realsense_camera_info.yaml',
                'image_rate': 10.0,
            }],
        ),
    ])
