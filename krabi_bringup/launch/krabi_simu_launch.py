from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

def generate_launch_description():
    isBlue_value = LaunchConfiguration('isBlue')
    xRobotPos_value = LaunchConfiguration('xRobotPos')
    yRobotPos_value = LaunchConfiguration('yRobotPos')
    zRobotOrientation_value = LaunchConfiguration('zRobotOrientation')
    
    isBlue_launch_arg = DeclareLaunchArgument(
        'isBlue',
        default_value='False'
    )
    xRobotPos_launch_arg = DeclareLaunchArgument(
        'xRobotPos',
        default_value='1.25'
    )
    yRobotPos_launch_arg = DeclareLaunchArgument(
        'yRobotPos',
        default_value='0.5'
    )
    zRobotOrientation_launch_arg = DeclareLaunchArgument(
        'zRobotOrientation',
        default_value='0.0'
    )

    tirette_spawn = Node(package='krabi_bringup',
              executable='tirette.py',
              output='both',
              namespace="krabi_ns",
              )
    
    odom_map_spawn = Node(package='tf2_ros',
              executable='static_transform_publisher',
              output='both',
              namespace="krabi_ns",
              arguments=["--x", "0", "--y", "0", "--z", "1", "--roll", "0", "--pitch", "0", "--yaw", "0",
                         "--child-frame-id", "odom", "--frame-id", "map"]
              )

    return LaunchDescription([isBlue_launch_arg, xRobotPos_launch_arg, yRobotPos_launch_arg, zRobotOrientation_launch_arg, tirette_spawn, odom_map_spawn,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('krabby_description'),
                    'launch',
                    'krabby_simulation.launch.py'
                ])
            ])
            ,launch_arguments={
                'isBlue': isBlue_value,
                'xRobotPos': xRobotPos_value,
                'yRobotPos': yRobotPos_value,
                'zRobotOrientation_value': zRobotOrientation_value
            }.items()
        )
    ])