from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    server_host = LaunchConfiguration("server_host")
    server_port = LaunchConfiguration("server_port")
    image_topic = LaunchConfiguration("image_topic")

    # THIS is exactly what ros2 launch turtlebot3_gazebo projects_empty_world.launch.py uses
    gazebo_pkg_share = get_package_share_directory("turtlebot3_gazebo")
    gazebo_launch = "./Turtlebot3_gestes/turtlebot3_gazebo/launch/projects/projects_empty_world.launch.py"


    return LaunchDescription([
        DeclareLaunchArgument("server_host", default_value="10.111.229.90"),
        DeclareLaunchArgument("server_port", default_value="9900"),
        DeclareLaunchArgument("image_topic", default_value="/rgb_camera/image"),

        # Same exports you used manually
        SetEnvironmentVariable("PROJECT_MODEL", "turtlebot3_burger_d435i"),
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),

        # Gazebo + RViz (EXACT same as manual command)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        ),

        # Robot roaming
        Node(
            package="turtlebot3_autonomous",
            executable="obstacle_avoider",
            name="movement_logic",
            output="screen",
        ),

        # Person detector (WSL → VM DeepLab)
        Node(
            package="ia_turtlebot_vision",
            executable="person_detector_node",
            name="person_detector",
            output="screen",
            parameters=[{
                "server_host": server_host,
                "server_port": server_port,
                "image_topic": image_topic,
                "send_every": 5,
                "jpeg_quality": 75,
                "min_area": 0.03,
                "socket_timeout": 4.0,
            }],
        ),

        # Hand signs
        Node(
            package="ia_turtlebot_vision",
            executable="gesture_detector_node",
            name="gesture_vision",
            output="screen",
        ),

        # Supervisor
        Node(
            package="robot_supervisor",
            executable="supervisor",
            name="brain",
            output="screen",
        ),
    ])
