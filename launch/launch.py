import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import launch_ros

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    config_path = get_share_file(
        package_name="lo_dev",
        file_name="config/config.yaml")

    rviz_config_file = get_share_file(
        package_name="lo_dev",
        file_name="config/rviz2.rviz")

    home_directory = os.path.expanduser("~")

    config_path_arg = DeclareLaunchArgument(
        "config_file",
        default_value=config_path,
        description="Path to config file for lo_dev"
    )

    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="integrated_to_init"
    )
    world_frame_arg = DeclareLaunchArgument(
        "world_frame",
        default_value="map",
    )
    sensor_frame_arg = DeclareLaunchArgument(
        "sensor_frame",
        default_value="sensor",
    )

    frontend_node = Node(
        package="lo_dev",
        executable="frontend_node",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[
            LaunchConfiguration("config_file"),
        ],
        remappings=[
            ("laser_odom_to_init", LaunchConfiguration("odom_topic")),
        ]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        # use_sim_time is automatically declared in ROS2 so set the param here (not in parameter.h)
        launch_ros.actions.SetParameter(name='use_sim_time', value='false'),
        config_path_arg,
        odom_topic_arg,
        world_frame_arg,
        sensor_frame_arg,
        frontend_node,
        rviz2_node,
    ])
    
