import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    # --- 1. Define Paths and Arguments ---


    orbslam3_pkg_share_dir = get_package_share_directory('orbslam3')

    vocab_path = os.path.join(
        orbslam3_pkg_share_dir,
        'vocabulary',
        'ORBvoc.txt'
    )
    settings_path = '/home/anas/ORB_SLAM3/Examples/Monocular-Inertial/AQUALOC/aqua.yaml'

    
    # Declare the node executable
    monocular_inertial_node = Node(
        package='orbslam3',
        executable='monocular_inertial', 
        name='orbslam3_node',
        output='screen',
        
        # GDB Prefix
        prefix=['gnome-terminal -- gdb -ex run --args'],

        parameters=[
            {
                "TimeshiftCamImu": -0.0403806549886,
                "vocabulary_path": vocab_path,
                "config_path": settings_path,
                "imu_topic": "/rtimulib_node/imu",
                "image_topic": "/camera/image_raw",
            }
        ],
        
        remappings=[
            ('/camera', '/camera/image_raw'),
            ('/imu', '/rtimulib_node/imu'),
        ],

    )

    return LaunchDescription([
        monocular_inertial_node
    ])
