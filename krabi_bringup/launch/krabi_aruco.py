from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    use_sim_time_value = LaunchConfiguration('use_sim_time')

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False'
    )

    # Launch arguments
    ref_frame_arg = DeclareLaunchArgument(
        'ref_frame',
        default_value='camera_link',
        description='Reference frame for marker pose'
    )



    # Node definition
    ros2_aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node',
        output='screen',
        namespace="krabi_ns",
        remappings=[
# Krabi simu
            #('/camera_info', '/krabi_ns/krabi_camera_info'),
#            ('/camera/image_raw', '/krabi_ns/krabi_cam_raw'),

# USB cam
#            ('/camera_info', '/image_raw/camera_info'),
#            ('/camera/image_raw', '/image_raw/compressed'),
#            ('/camera/image_raw', '/image_raw'),

# rpi5
            #('/camera_info', '/camera/camera_info'),
            #('/camera/image_raw', '/camera/image_raw'),
            ('/camera/camera_info', 'krabi_cam/camera_info'),
            ('/camera/image_raw', 'krabi_cam/image_raw'),

        ],
        parameters=[
            {
                'marker_size': 0.04,
                'aruco_dictionary_id': "DICT_4X4_50",
                'camera_frame': LaunchConfiguration('ref_frame'),
                "use_sim_time": use_sim_time_value
            }
        ],
    )

    return LaunchDescription([
        ref_frame_arg,
        ros2_aruco_node
    ])
