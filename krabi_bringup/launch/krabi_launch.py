from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
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
    do_record_value = LaunchConfiguration('do_record')
    use_aruco_value = LaunchConfiguration('use_aruco')
    use_caisse_detector_value = LaunchConfiguration('use_caisse_detector')

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
        default_value='1.570796327'
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

    do_record_launch_arg = DeclareLaunchArgument(
        'do_record',
        default_value='False'
    )

    use_aruco_launch_arg = DeclareLaunchArgument(
        'use_aruco',
        default_value='False'
    )

    use_caisse_detector_launch_arg = DeclareLaunchArgument(
        'use_caisse_detector',
        default_value='False'
    )

    odom_map_spawn = Node(package='tf2_ros',
        executable='static_transform_publisher',
        output='both',
        namespace="krabi_ns",
        arguments=["--x", xRobotPos_value, "--y", yRobotPos_value, "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", zRobotOrientation_value,
                    "--child-frame-id", "odom", "--frame-id", "map"],
        condition=IfCondition(PythonExpression(["not ", use_lidar_loc_value])),
        parameters=[{"use_sim_time": isSimulation_value}]
    )

    grabi_base_link_spawn = Node(package='tf2_ros',
        executable='static_transform_publisher',
        output='both',
        namespace="krabi_ns",
        arguments=["--x", "0.17", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--child-frame-id", "grabi", "--frame-id", "base_link"],
        parameters=[{"use_sim_time": isSimulation_value}]
    )

    billig_base_link_spawn = Node(package='tf2_ros',
        executable='static_transform_publisher',
        output='both',
        namespace="krabi_ns",
        arguments=["--x", "0", "--y", "-0.2", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--child-frame-id", "billig", "--frame-id", "base_link"],
        parameters=[{"use_sim_time": isSimulation_value}]
    )

    #<node pkg="tf" type="static_transform_publisher" name="base_link_suction_cup" args="0.2 0 0 0 0 0 krabby/suction_cup krabby/base_link 50"/>

    record = ExecuteProcess(condition=IfCondition(do_record_value),
        cwd="/var/log/krabi/", cmd=['ros2', 'bag', 'record', '-a'], output='screen', log_cmd=True,
    )

    launch_description = LaunchDescription([isSimulation_launch_arg, billig_base_link_spawn, do_record_launch_arg, use_aruco_launch_arg, use_caisse_detector_launch_arg, isBlue_launch_arg, xRobotPos_launch_arg, yRobotPos_launch_arg, 
                                            zRobotOrientation_launch_arg, use_lidar_loc_launch_arg, 
                                            can_hardware_launch_arg, odom_map_spawn, grabi_base_link_spawn, record, 

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
                'zRobotOrientation_value': zRobotOrientation_value,
                "use_sim_time": isSimulation_value
            }.items(),
            condition=IfCondition(isSimulation_value)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('krabi_bringup'),
                    'launch',
                    'krabi_amcl.py'
                ])
            ])
            ,launch_arguments={
                'isBlue': isBlue_value,
                'xRobotPos': xRobotPos_value,
                'yRobotPos': yRobotPos_value,
                'zRobotOrientation_value': zRobotOrientation_value,
                "use_sim_time": isSimulation_value

            }.items(),
            condition=IfCondition(use_lidar_loc_value)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('krabi_bringup'),
                    'launch',
                    'krabi_aruco.py'
                ])
            ])
            ,launch_arguments={
            }.items(),
            condition=IfCondition(use_aruco_value)
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
                'can_hardware': can_hardware_value,
                'init_pose/x': xRobotPos_value,
                'init_pose/y': yRobotPos_value,
                'init_pose/theta': zRobotOrientation_value,
                "use_sim_time": isSimulation_value,
                "use_camera": use_aruco_value or use_caisse_detector_value
            }.items(),
            condition=UnlessCondition(isSimulation_value)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('krabi_bringup'),
                    'launch',
                    'transforms_launchKV2.py' # transforms_launch.py for KV1
                ])
            ]),
            condition=UnlessCondition(isSimulation_value),
            launch_arguments={
                "use_sim_time": isSimulation_value
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('krabi_bringup'),
                    'launch',
                    'krabi_main_launch.py'
                ])
            ]),
            launch_arguments={
                "use_sim_time": isSimulation_value
            }.items()
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
                'publish_tf_odom': "False",
                'init_pose/x': xRobotPos_value,
                'init_pose/y': yRobotPos_value,
                'init_pose/theta': zRobotOrientation_value,
                "use_sim_time": isSimulation_value
            }.items(),
            condition=IfCondition(isSimulation_value)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('krabi_caisse_detector'),
                    'launch',
                    'caisse_detector_launch.py'
                ])
            ]),
            launch_arguments={
                'isBlue': isBlue_value,
                "use_sim_time": isSimulation_value
            }.items(),
            condition=IfCondition(use_caisse_detector_value)
        ) 
    ])



    return launch_description
