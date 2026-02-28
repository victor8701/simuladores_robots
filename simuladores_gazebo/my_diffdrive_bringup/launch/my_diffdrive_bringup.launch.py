# Version: 1.1.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- PATHS ---
    pkg_bringup = get_package_share_directory('my_diffdrive_bringup')
    pkg_description = get_package_share_directory('my_diffdrive_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to the SDF model file (Description Package)
    # Assumes file is at: src/my_diffdrive_description/models/model.sdf
    sdf_file_path = os.path.join(pkg_description, 'models', 'model.sdf')
    
    # Path to the World file (Bringup Package)
    world_file_path = os.path.join(pkg_bringup, 'worlds', 'world_test_esq_ext.sdf')

    # --- NODES ---

    # 1. Launch Gazebo (The World)
    # We pass the world file directly to the gz_args
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r --headless-rendering {world_file_path}'}.items(),
    )

    # 2. Spawn the Robot (The Model)
    # Since it is SDF, we use '-file' instead of '-topic'
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_diffdrive_robot',
            '-file', sdf_file_path,
            '-x', '12.5', '-y', '10.8', '-z', '0.2'
        ],
        output='screen'
    )

    # 3. ROS-Gazebo Bridge (The Communication)
    # Bridges the Gazebo topics to ROS 2 topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Command Velocity: ROS -> Gazebo
            '/model/my_diffdrive_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            
            # Redundant Odometry & Positioning: Gazebo -> ROS
            '/model/my_diffdrive_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/my_diffdrive_robot/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/my_diffdrive_robot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

            # Proximity Sensors: Gazebo -> ROS
            '/ps0@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ps1@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ps2@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ps3@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ps4@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ps5@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ps6@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ps7@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # IMU: Gazebo -> ROS
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        bridge
    ])