from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    use_tim_instead_of_neato = LaunchConfiguration('use_tim_instead_of_neato', default='True')
    use_lidar_loc = LaunchConfiguration('use_lidar_loc', default='False')

    neato_laser_publisher_node = Node(
        package='xv_11_laser_driver',
        executable='neato_laser_publisher',
        name='neato_laser_publisher',
        namespace="krabi_ns",
        parameters=[{'port': '/dev/lidar'}, {'baud': 115200}, {'frame_id': 'krabby/neato_laser'}],
        condition=UnlessCondition(use_tim_instead_of_neato)
    )

    tim_obstacles_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('krabi_bringup'),
                'launch',
                'tim_obstacles_launch.py'
            ])
        ])
        ,condition=IfCondition(use_tim_instead_of_neato)
    )

    tim_loc_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('krabi_bringup'),
                'launch',
                'tim_loc_launch.py'
            ])
        ])
        ,condition=IfCondition(use_lidar_loc)
    )

    return LaunchDescription([
        #neato_laser_publisher_node,
        tim_obstacles_group
        ,tim_loc_group
    ])
