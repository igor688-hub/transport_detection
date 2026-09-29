import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lidar_obstacle_detector')
    config = LaunchConfiguration('config')
    bag = LaunchConfiguration('bag')
    rviz = LaunchConfiguration('rviz')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'params.yaml')),
        DeclareLaunchArgument('input_topic', default_value='/lidar_points'),
        DeclareLaunchArgument('log_path', default_value=''),
        DeclareLaunchArgument('bag', default_value=''),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(
            package='lidar_obstacle_detector',
            executable='detector_node',
            name='lidar_obstacle_detector',
            output='screen',
            parameters=[{
                'config': config,
                'input_topic': LaunchConfiguration('input_topic'),
                'log_path': LaunchConfiguration('log_path'),
            }],
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag, '--rate', LaunchConfiguration('rate')],
            output='screen',
            condition=IfCondition(PythonExpression(["'", bag, "' != ''"])),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'detector.rviz')],
            condition=IfCondition(rviz),
        ),
    ])
