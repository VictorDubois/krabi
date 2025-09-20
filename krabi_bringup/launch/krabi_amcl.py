from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory

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


    amcl_spawn = Node(package='nav2_amcl',
              executable='amcl',
              output='both',
              namespace="krabi_ns",
              parameters=[{"scan_topic": 'scan_loc',
                           "map_topic": "/map",
                           "set_initial_pose": True,
                           "base_frame_id": "base_link",
                           "tf_broadcast": False, #True si on veut utiliser la loc lidar sans EKF
                           "laser_min_range": 0.3,
                           "update_min_d": 0.02,
                           "update_min_a": 0.02,
                           "initial_pose": {
                                "x": xRobotPos_value,
                                "y": yRobotPos_value,
                                "z": 0.0,
                                "yaw": zRobotOrientation_value
                            }}]
              )
    
    robot_localization_spawn = Node(package='robot_localization',
              executable='ekf_node',
              output='both',
              namespace="krabi_ns",
              parameters=[{"initial_state": [xRobotPos_value,  yRobotPos_value, 0.0,
                                 0.0,  0.0,  zRobotOrientation_value,
                                 0.0,  0.0,  0.0,
                                 0.0,  0.0,  0.0,
                                 0.0,  0.0,  0.0]}
                            ,PathJoinSubstitution([FindPackageShare('krabi_bringup'), 'params', 'ekf_template_krabi.yaml'])
                        ]
              )

    map_server_spawn = Node(package='nav2_map_server',
              executable='map_server',
              output='both',
              namespace="krabi_ns",
              parameters=[{'yaml_filename': os.path.join(get_package_share_directory("krabi_bringup"), 'map','pixel_art_2026.yaml'),
                            "topic_name": "/map"}],
              )
    
    lifecycle_autostart_spawn = Node(package='nav2_lifecycle_manager',
              executable='lifecycle_manager',
              output='both',
              namespace="krabi_ns",
              parameters=[{'node_names': ["map_server", "amcl"], "autostart": True}]
              )

    return LaunchDescription([isBlue_launch_arg, xRobotPos_launch_arg, yRobotPos_launch_arg, zRobotOrientation_launch_arg, amcl_spawn,map_server_spawn, lifecycle_autostart_spawn, robot_localization_spawn])