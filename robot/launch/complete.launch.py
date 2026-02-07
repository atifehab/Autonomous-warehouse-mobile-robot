from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robot_share = get_package_share_directory('robot')
    
    # Static transforms for sensors
    # base_link -> imu_link (adjust x, y, z based on your actual IMU position)
    imu_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf_publisher',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
    )
    
    # base_link -> laser_frame (adjust based on your LIDAR position)
    # Format: x y z yaw pitch roll parent_frame child_frame
    laser_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_publisher',
        arguments=['0.1', '0', '0.15', '0', '0', '0', 'base_link', 'laser_frame']
    )
    
    # Robot State Publisher (publishes robot URDF)
    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(
                os.path.join(robot_share, 'urdf', 'robot.urdf')
            ).read()
        }]
    )
    
    # EKF for sensor fusion
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(robot_share, 'config', 'ekf.yaml')]
    )
    
    # RPLIDAR
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'frame_id': 'laser_frame',  # IMPORTANT: Match with static transform
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }],
        output='screen'
    )
    
    # SLAM Toolbox
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[os.path.join(robot_share, 'config', 'slam.yaml')],
        output='screen'
    )
    
    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(robot_share, 'urdf.rviz')]
    )
    
    return LaunchDescription([
        imu_transform,
        laser_transform,
        robot_description,
        ekf,
        rplidar,
        slam,
        rviz
    ])