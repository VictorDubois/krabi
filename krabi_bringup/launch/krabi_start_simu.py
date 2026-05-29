from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    xRobotPos           = LaunchConfiguration('xRobotPos')
    yRobotPos           = LaunchConfiguration('yRobotPos')
    zRobotOrientation   = LaunchConfiguration('zRobotOrientation')
    isBlue              = LaunchConfiguration('isBlue')
    use_aruco           = LaunchConfiguration('use_aruco')
    use_caisse_detector = LaunchConfiguration('use_caisse_detector')
    use_lidar_loc       = LaunchConfiguration('use_lidar_loc')
    gui                 = LaunchConfiguration('gui')
    # Named gz_world (not 'world') to avoid polluting the 'world' launch argument
    # that spawn_and_bridge.launch.py uses for the Gazebo service name auto-detection.
    # spawn_world.py uses 'world' for the world *file* name; spawn_and_bridge uses
    # 'world' for the world *SDF name* (default = '').  The two-terminal approach
    # works because they run in separate launch processes with separate contexts.
    gz_world          = LaunchConfiguration('gz_world')

    pkg = get_package_share_directory('krabi_description')
    os.environ['GZ_SIM_RESOURCE_PATH'] = pkg + '/models:' + pkg + '/worlds'

    kill_gz = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -f "gz sim" 2>/dev/null; sleep 1'],
        output='screen'
    )

    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', gz_world, '-v', '-r'],
        output='screen',
        condition=IfCondition(gui)
    )

    gz_headless = ExecuteProcess(
        cmd=['gz', 'sim', gz_world, '-v', '-r', '-s', '--headless-rendering'],
        output='screen',
        condition=UnlessCondition(gui)
    )

    # Polls every second until the Gazebo world service appears, then exits 0.
    # OnProcessExit fires regardless of exit code, so robot_launch starts once this finishes.
    wait_for_gz = ExecuteProcess(
        cmd=['bash', '-c',
             'until gz service -l 2>/dev/null | grep -q "^/world/"; do sleep 1; done'
             ' && echo "[krabiStart] Gazebo ready, launching robot stack"'],
        output='screen'
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('krabi_bringup'), 'launch', 'krabi_launch.py'
            ])
        ]),
        launch_arguments={
            'xRobotPos':         xRobotPos,
            'yRobotPos':         yRobotPos,
            'zRobotOrientation': zRobotOrientation,
            'isBlue':            isBlue,
            'use_aruco':         use_aruco,
            'use_caisse_detector': use_caisse_detector,
            'use_lidar_loc':     use_lidar_loc,
            'isSimulation':      'True',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('xRobotPos',         default_value='-1.25'),
        DeclareLaunchArgument('yRobotPos',         default_value='-0.75'),
        DeclareLaunchArgument('zRobotOrientation', default_value='1.570796327'),
        DeclareLaunchArgument('isBlue',            default_value='True'),
        DeclareLaunchArgument('use_aruco',         default_value='False'),
        DeclareLaunchArgument('use_caisse_detector', default_value='False'),
        DeclareLaunchArgument('use_lidar_loc',     default_value='False'),
        DeclareLaunchArgument('gui',               default_value='False'),
        DeclareLaunchArgument('gz_world',          default_value='table2026.world'),
        kill_gz,
        RegisterEventHandler(
            OnProcessExit(
                target_action=kill_gz,
                on_exit=[
                    gz_gui,
                    gz_headless,
                    wait_for_gz,
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=wait_for_gz,
                            on_exit=[robot_launch],
                        )
                    ),
                ]
            )
        ),
    ])
