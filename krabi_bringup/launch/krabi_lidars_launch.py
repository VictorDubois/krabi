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
    use_ld_lidar_instead_of_tim = LaunchConfiguration('use_ld_lidar_instead_of_tim', default='True')
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
        ,condition=UnlessCondition(use_ld_lidar_instead_of_tim)
    )

    ldlidar_obstacles_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ldlidar'),
                'launch',
                'ldlidar.launch.py'
            ])
        ])
        ,launch_arguments={
                'serial_port': "/dev/ttyUSB0",
                'topic_name': "/krabi_ns/scan_obstacles",
                'lidar_frame': "ldlidar_top",
                'range_threshold': "0.05"
            }.items()
        ,condition=IfCondition(use_ld_lidar_instead_of_tim)
    )

    ldlidar_tim_spawn = Node(package='tf2_ros',
        executable='static_transform_publisher',
        output='both',
        namespace="krabi_ns",
        arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "-1.570796327",
                    "--child-frame-id", "ldlidar_top", "--frame-id", "tim_top"]
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
        #,tim_loc_group
        ,ldlidar_obstacles_group
        ,ldlidar_tim_spawn
    ])

