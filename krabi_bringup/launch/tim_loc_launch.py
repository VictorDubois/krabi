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
                    {'frame_id': 'tim_bottom'},
                    {'laserscan_topic': 'scan_loc'},
                    {'cloud_topic': "cloud_loc"},
                    {'min_ang': -1.77},
                    {'max_ang': 2.18},
                    {"tf_base_frame_id": "base_link"},
                    {"tf_base_lidar_xyz_rpy": "-0.07,0.091,0.06,3.14159,0,2.35619449"},
                    {"tf_publish_rate": 20}],
        #remappings=[
        #    ('sick_tim_5xx/imu', 'sick_obstacle/imu'),
        #    ('sick_tim_5xx/encoder', 'sick_obstacle/encoder')
        #]
    )

    return LaunchDescription([
        tim_loc
    ])

