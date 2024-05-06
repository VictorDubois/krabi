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
    motors_node = Node(
        package='krabi_python_serial_broker',
        executable='simple_stm32_broker.py',
        name='motors_node',
        namespace="krabi_ns",
        parameters=[{'port': '/dev/motors'}, {'baud': 115200}]
    )

    servos_node = Node(
        package='krabi_python_serial_broker',
        executable='simple_arduino_broker.py',
        name='servos_node',
        namespace="krabi_ns",
        parameters=[{'port': '/dev/servos'}, {'baud': 57600}]
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('krabi_gpio'),
                    'launch',
                    'krabi_gpio_launch.py'
                ])
            ])
        ),
        motors_node,
        servos_node
    ])
