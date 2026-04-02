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
    xRobotPos_value = LaunchConfiguration('xRobotPos')
    yRobotPos_value = LaunchConfiguration('yRobotPos')
    zRobotOrientation_value = LaunchConfiguration('zRobotOrientation')
    use_camera_value = LaunchConfiguration('use_camera')

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
    use_camera_launch_arg = DeclareLaunchArgument(
        'use_camera',
        default_value="False"
    )

    can_hardware_value = LaunchConfiguration('can_hardware')
    
    can_hardware_launch_arg = DeclareLaunchArgument(
        'can_hardware',
        default_value='True'
    )
    #gpio_kraboss_node = Node(
    #    package='gpio_kraboss',
    #    executable='main.py',
    #    name='gpio_kraboss_node'
    #)

    motors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('krabi_fast_serial_broker'),
                'launch',
                'fast_serial_broker_STM32_launch.py'
            ])
        ])
        ,condition=UnlessCondition(can_hardware_value)
    )

    gpio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('krabi_gpio'),
                'launch',
                'krabi_gpio_launch.py'
            ])
        ])
        #,condition=UnlessCondition(can_hardware_value)# To test on my PC
    )

    servos_node = Node(
        package='krabi_python_serial_broker',
        executable='simple_arduino_broker.py',
        name='servos_node',
        namespace="krabi_ns",
        parameters=[{'port': '/dev/servos'}, {'baud': 57600}],
        condition=UnlessCondition(can_hardware_value)
    )

    motors_can_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('krabi_can_broker'),
                'launch',
                'can_broker_stm32_launch.py'
            ])
        ])
        ,launch_arguments={
            'publish_tf_odom': "True",
            'init_pose/x': xRobotPos_value,
            'init_pose/y': yRobotPos_value,
            'init_pose/theta': zRobotOrientation_value
        }.items()
        ,condition=IfCondition(can_hardware_value)
    )

    actuators_can_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('krabi_can_broker'),
                'launch',
                'can_broker_actuators_launch.py'
            ])
        ])
        ,condition=IfCondition(can_hardware_value)
    )

    usb_camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        output='screen',
        namespace="krabi_ns",
        remappings=[
            ('camera_info', 'krabi_cam/camera_info'),
            ('image_raw', 'krabi_cam/image_raw')
    ])

    camera_ros_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node',
        output='screen',
        namespace="krabi_ns",
        remappings=[
            ('camera/camera_info', 'krabi_cam/camera_info'),
            ('camera/image_raw', 'krabi_cam/image_raw')],
        condition=IfCondition(use_camera_value)
    )

    return LaunchDescription([
        can_hardware_launch_arg,
        xRobotPos_launch_arg,
        yRobotPos_launch_arg,
        zRobotOrientation_launch_arg,
        use_camera_launch_arg,

        gpio_launch,
        motors_launch,
        servos_node,
        motors_can_launch,
        actuators_can_launch,
        #usb_camera_node,
        camera_ros_node
    ])
