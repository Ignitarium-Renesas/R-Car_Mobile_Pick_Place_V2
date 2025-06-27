# rcar_robot/launch/rcar_bringup.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    imu_filter_launch = os.path.join(
        get_package_share_directory('imu_complementary_filter'),
        'launch/complementary_filter.launch.py'
    )

    static_tf_launch = os.path.join(
        get_package_share_directory('static_tf'),
        'launch/static_tf.launch.py'
    )

    ekf_config_path = os.path.join(
        get_package_share_directory("rcar_robot"),
        'params',
        'ekf_rcar.yaml'
    )

    return LaunchDescription([
        # rcar odometry node
        Node(
            package='rcar_robot',
            executable='rcar_odom',
            name='rcar_odom',
            output='screen'
        ),

        # IMU complementary filter
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_filter_launch)
        ),

        # Static TFs
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(static_tf_launch)
        ),

        # EKF node directly from config
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),

        # Laser scan filter chain
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("rcar_robot"),
                    "params",
                    "box_filter.yaml"
                ])
            ],
            remappings=[
                ('scan', '/scan_raw'),         # Input topic
                ('scan_filtered', '/scan')     # Output topic
            ]
        )
    ])
