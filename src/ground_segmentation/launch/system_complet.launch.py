import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # =========================
    # DDS ENV (match robot)
    # =========================
    env_vars = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', '1'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
    ]

    # =========================
    # Launch arguments
    # =========================
    declare_max_hz = DeclareLaunchArgument('max_hz', default_value='5.0')
    declare_use_packet_stamp = DeclareLaunchArgument('use_packet_stamp', default_value='true')
    declare_extra_pitch = DeclareLaunchArgument('extra_pitch_deg', default_value='19.0')
    declare_z_offset = DeclareLaunchArgument('z_offset', default_value='0.13')

    max_hz = LaunchConfiguration('max_hz')
    use_packet_stamp = LaunchConfiguration('use_packet_stamp')
    extra_pitch = LaunchConfiguration('extra_pitch_deg')
    z_offset = LaunchConfiguration('z_offset')

    # =========================
    # RViz config (your choice)
    # =========================
    rviz_config = os.path.join(
        get_package_share_directory('ground_segmentation'),
        'rviz',
        'grid.rviz'  # change if your file name differs
    )
    # If you don't have grid.rviz, use turtlebot3_gazebo config:
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'rviz',
            'tb3_gazebo.rviz'
        )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_config]
    )

    # =========================
    # Decompressor
    # =========================
    decompress_node = Node(
        package='combined_decompressor',
        executable='rcmb_receiver',
        name='rcmb_receiver',
        output='screen',
        parameters=[{
            'in_topic': '/camera/combined',
            'out_image': '/camera/color/image_raw_decomp',
            'out_cloud': '/camera/depth/color/points_decomp',
            'colorize': True,
            'max_hz': max_hz,
            'use_packet_stamp': use_packet_stamp,
            'extra_pitch_deg': extra_pitch,
            'z_offset': z_offset,
            'color_frame': 'base_link',
            'cloud_frame': 'base_link',
        }],
    )

    # =========================
    # Ground segmentation / grid
    # =========================
    grid_node = Node(
        package='ground_segmentation',
        executable='grid_projection_node',
        name='smart_grid_node',
        output='screen',
        parameters=[{
            'cloud_topic': '/camera/depth/color/points_decomp',
            'use_sim_time': False,
        }]
    )

    return LaunchDescription(
        env_vars + [
            declare_max_hz,
            declare_use_packet_stamp,
            declare_extra_pitch,
            declare_z_offset,
            rviz_node,
            decompress_node,
            grid_node,
        ]
    )
