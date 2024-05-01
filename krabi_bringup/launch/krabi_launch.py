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
    
    isBlue_launch_arg = DeclareLaunchArgument(
        'isBlue',
        default_value='False'
    )
    xRobotPos_launch_arg = DeclareLaunchArgument(
        'xRobotPos',
        default_value='-1.25'
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
        default_value='true'
    )
    use_lidar_loc_launch_arg = DeclareLaunchArgument(
        'use_lidar_loc',
        default_value='False'
    )

    odom_map_spawn = Node(package='tf2_ros',
        executable='static_transform_publisher',
        output='both',
        namespace="krabi_ns",
        arguments=["--x", xRobotPos_value, "--y", yRobotPos_value, "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", zRobotOrientation_value,
                    "--child-frame-id", "odom", "--frame-id", "map"],
        condition=IfCondition(PythonExpression(["not ", use_lidar_loc_value, " and not ", isSimulation_value]))
    )

    

    launch_description = LaunchDescription([isBlue_launch_arg, xRobotPos_launch_arg, yRobotPos_launch_arg, zRobotOrientation_launch_arg,
                                            isSimulation_launch_arg, use_lidar_loc_launch_arg, odom_map_spawn,
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
                'use_lidar_loc': use_lidar_loc_value
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
                'publish_tf_odom': "true",
                'init_pose/x': xRobotPos_value,
                'init_pose/y': yRobotPos_value,
                'init_pose/theta': zRobotOrientation_value
            }.items(),
            condition=UnlessCondition(isSimulation_value)
        ) 
    ])



    return launch_description
