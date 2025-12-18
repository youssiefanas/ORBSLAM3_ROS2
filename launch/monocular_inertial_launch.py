import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    vocab_path_default = os.path.join(
        get_package_share_directory('orbslam3'), 
        'vocabulary', 'ORBvoc.txt'
    )
    # /home/anas/ORB_SLAM3/Examples/Monocular-Inertial/AQUALOC/aqua.yaml
    config_path_default = '/home/youssief/jazzy_ws/src/ORBSLAM3_ROS2/config/monocular-inertial/oceansim_camera_imu.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(
            'vocabulary_path',
            default_value=vocab_path_default,
            description='Path to ORBvoc.txt'
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value=config_path_default,
            description='Path to the configuration YAML file'
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/oceansim/robot/imu',
            description='IMU topic name'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/oceansim/robot/uw_img',
            description='Image topic name'
        ),
        DeclareLaunchArgument(
            'use_compressed',
            default_value='true',
            description='Use compressed images (true/false)'
        ),
        DeclareLaunchArgument(
            'trial_name',
            default_value='',
            description='Optional name for the trial/session'
        ),
        Node(
            package='orbslam3',
            executable='monocular_inertial',
            name='monocular_inertial_node',
            output='screen',
            parameters=[{
                'vocabulary_path': LaunchConfiguration('vocabulary_path'),
                'config_path': LaunchConfiguration('config_path'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'image_topic': LaunchConfiguration('image_topic'),
                'use_compressed': LaunchConfiguration('use_compressed'),
                'trial_name': LaunchConfiguration('trial_name')
            }],
        )
    ])
