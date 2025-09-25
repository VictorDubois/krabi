from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    LD06_top_base_link_spawn = Node(package='tf2_ros',
        executable='static_transform_publisher',
        output='both',
        namespace="krabi_ns",
        arguments=["--x", "0", "--y", "0", "--z", "0.38", "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--child-frame-id", "ldlidar_top", "--frame-id", "base_link"]
        )
    
    tim_bottom_base_link_spawn = Node(package='tf2_ros',
        executable='static_transform_publisher',
        output='both',
        namespace="krabi_ns",
        arguments=["--x", "-0.07", "--y", "0.091", "--z", "0.06", "--roll", "3.14159", "--pitch", "0", "--yaw", "2.35619449",
                    "--child-frame-id", "tim_bottom", "--frame-id", "base_link"]
        )

    return LaunchDescription([
        LD06_top_base_link_spawn,
        tim_bottom_base_link_spawn
    ])
