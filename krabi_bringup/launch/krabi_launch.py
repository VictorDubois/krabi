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
    isBlue_value = LaunchConfiguration('isBlue')
    xRobotPos_value = LaunchConfiguration('xRobotPos')
    yRobotPos_value = LaunchConfiguration('yRobotPos')
    zRobotOrientation_value = LaunchConfiguration('zRobotOrientation')
    isSimulation_value = LaunchConfiguration('isSimulation')
    use_lidar_loc_value = LaunchConfiguration('use_lidar_loc')
    can_hardware_value = LaunchConfiguration('can_hardware')
    
    isBlue_launch_arg = DeclareLaunchArgument(
        'isBlue',
        default_value='False'
    )
    xRobotPos_launch_arg = DeclareLaunchArgument(
        'xRobotPos',
        default_value='-0.75'
    )
    yRobotPos_launch_arg = DeclareLaunchArgument(
        'yRobotPos',
        default_value='-0.75'
    )
    zRobotOrientation_launch_arg = DeclareLaunchArgument(
        'zRobotOrientation',
        default_value='0.0'
    )
    isSimulation_launch_arg = DeclareLaunchArgument(
        'isSimulation',
        default_value='True'
    )
    use_lidar_loc_launch_arg = DeclareLaunchArgument(
        'use_lidar_loc',
        default_value='False'
    )

    can_hardware_launch_arg = DeclareLaunchArgument(
        'can_hardware',
        default_value='True'
    )

    odom_map_spawn = Node(package='tf2_ros',
        executable='static_transform_publisher',
        output='both',
        namespace="krabi_ns",
        arguments=["--x", xRobotPos_value, "--y", yRobotPos_value, "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", zRobotOrientation_value,
                    "--child-frame-id", "odom", "--frame-id", "map"],
        condition=IfCondition(PythonExpression(["not ", use_lidar_loc_value]))
    )

    grabi_base_link_spawn = Node(package='tf2_ros',
        executable='static_transform_publisher',
        output='both',
        namespace="krabi_ns",
        arguments=["--x", "0.17", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--child-frame-id", "grabi", "--frame-id", "base_link"]
    )
    #<node pkg="tf" type="static_transform_publisher" name="base_link_suction_cup" args="0.2 0 0 0 0 0 krabby/suction_cup krabby/base_link 50"/>

    

    launch_description = LaunchDescription([isBlue_launch_arg, xRobotPos_launch_arg, yRobotPos_launch_arg, zRobotOrientation_launch_arg,
                                            isSimulation_launch_arg, use_lidar_loc_launch_arg, can_hardware_launch_arg, odom_map_spawn, grabi_base_link_spawn,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('krabi_bringup'),
                    'launch',
                    'krabi_simu_launch.py'
                ])
            ])
            ,launch_arguments={
                'isBlue': isBlue_value,
                'xRobotPos': xRobotPos_value,
                'yRobotPos': yRobotPos_value,
                'zRobotOrientation_value': zRobotOrientation_value
            }.items(),
                        condition=IfCondition(isSimulation_value)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('krabi_bringup'),
                    'launch',
                    'krabi_hardware_launch.py'
                ])
            ])
            ,launch_arguments={
                'use_lidar_loc': use_lidar_loc_value,
                'can_hardware': can_hardware_value
            }.items(),
            condition=UnlessCondition(isSimulation_value)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('krabi_bringup'),
                    'launch',
                    'transforms_launch.py'
                ])
            ]),
            condition=UnlessCondition(isSimulation_value)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('krabi_bringup'),
                    'launch',
                    'krabi_main_launch.py'
                ])
            ])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('main_strategy'),
                    'launch',
                    'odom_TF_pub_launch.py'
                ])
            ]),
            launch_arguments={
                'publish_tf_odom': "True",
                'init_pose/x': xRobotPos_value,
                'init_pose/y': yRobotPos_value,
                'init_pose/theta': zRobotOrientation_value
            }.items(),
            condition=UnlessCondition(isSimulation_value)
        ) 
    ])



    return launch_description
