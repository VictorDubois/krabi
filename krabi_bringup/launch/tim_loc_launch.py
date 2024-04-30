from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tim_loc = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='tim_loc',
        namespace="krabi_ns",
        parameters=[{"scanner_type": "sick_tim_5xx"},
                    {'hostname': '192.168.20.16'},
                    {'frame_id': 'krabby/tim_bottom'},
                    {'laserscan_topic': 'scan_loc'},
                    {'cloud_topic': "cloud_loc"}],
        #remappings=[
        #    ('sick_tim_5xx/imu', 'sick_obstacle/imu'),
        #    ('sick_tim_5xx/encoder', 'sick_obstacle/encoder')
        #]
    )

    return LaunchDescription([
        tim_loc
    ])
