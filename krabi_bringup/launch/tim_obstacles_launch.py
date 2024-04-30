from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    tim_obstacles = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='tim_obstacles',
        namespace="krabi_ns",
        parameters=[{"scanner_type": "sick_tim_5xx"},
                    {'hostname': '192.168.0.17'},
                    {'frame_id': 'krabby/tim_top'},
                    {'laserscan_topic': 'scan_obstacles'},
                    {'cloud_topic': "cloud_obstacles"}],
        #remappings=[
        #    ('sick_tim_5xx/imu', 'sick_obstacle/imu'),
        #    ('sick_tim_5xx/encoder', 'sick_obstacle/encoder')
        #]
    )

    return LaunchDescription([
        tim_obstacles
    ])
