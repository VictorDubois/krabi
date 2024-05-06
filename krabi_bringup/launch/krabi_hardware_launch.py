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
    )

    gpio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('krabi_gpio'),
                'launch',
                'krabi_gpio_launch.py'
            ])
        ])
    )

    servos_node = Node(
        package='krabi_python_serial_broker',
        executable='simple_arduino_broker.py',
        name='servos_node',
        namespace="krabi_ns",
        parameters=[{'port': '/dev/servos'}, {'baud': 57600}]
    )

    return LaunchDescription([
        gpio_launch,
        motors_launch,
        servos_node
    ])
