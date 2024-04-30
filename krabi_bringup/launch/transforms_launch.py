from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tim_top_base_link_spawn = Node(package='tf2_ros',
        executable='static_transform_publisher',
        output='both',
        namespace="krabi_ns",
        arguments=["--x", "0", "--y", "0", "--z", "0.3", "--roll", "0", "--pitch", "0", "--yaw", "1.5708",
                    "--child-frame-id", "tim_top", "--frame-id", "base_link"]
        )
    
    tim_bottom_base_link_spawn = Node(package='tf2_ros',
        executable='static_transform_publisher',
        output='both',
        namespace="krabi_ns",
        arguments=["--x", "-0.112", "--y", "0", "--z", "0.06", "--roll", "0", "--pitch", "3.14159", "--yaw", "0",
                    "--child-frame-id", "tim_bottom", "--frame-id", "base_link"]
        )

    return LaunchDescription([
        tim_top_base_link_spawn,
        tim_bottom_base_link_spawn
    ])
