import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    # --- 1. Define Paths and Arguments ---


    orbslam3_pkg_share_dir = get_package_share_directory('orbslam3')

    vocab_path = '/home/anas/kilted_ws/src/ORBSLAM3_ROS2/vocabulary/ORBvoc.txt'

    settings_path = '/home/anas/ORB_SLAM3/Examples/Monocular-Inertial/AQUALOC/aqua.yaml'

    
    # Declare the node executable
    monocular_inertial_node = Node(
        package='orbslam3',
        executable='monocular_inertial', 
        name='orbslam3_node',
        output='screen',
        
        parameters=[
                {"TimeshiftCamImu": -0.0403806549886},
        ],
        # --- 2. Pass Positional Arguments ---
        # Positional arguments must be passed as a list of strings
        arguments=[
            TextSubstitution(text=vocab_path),
            TextSubstitution(text=settings_path),
        ],

        # --- 3. Topic Remappings ---
        # -r /camera:=/camera/image_raw
        # -r /imu:=/rtimulib_node/imu
        remappings=[
            ('/camera', '/camera/image_raw'),
            ('/imu', '/rtimulib_node/imu'),
        ],

        # Optional: Set required QoS profiles if necessary
        # ros_arguments=[
        #     '--ros-args', 
        #     '--remap', 'topic_name:=new_topic', 
        #     '--qos-profile', '/camera:sensor_data' 
        # ],

    )

    return LaunchDescription([
        monocular_inertial_node
    ])