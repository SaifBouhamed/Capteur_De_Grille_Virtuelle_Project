from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    server_host = LaunchConfiguration("server_host")
    server_port = LaunchConfiguration("server_port")
    image_topic = LaunchConfiguration("image_topic")

    # Recherche dynamique du chemin de Gazebo pour éviter les erreurs de dossier
    gazebo_pkg_share = get_package_share_directory("turtlebot3_gazebo")
    
    # On utilise os.path.join pour être plus robuste sur les chemins
    gazebo_launch = os.path.join(gazebo_pkg_share, 'launch', 'projects', 'projects_empty_world.launch.py')

    return LaunchDescription([
        # --- SOLUTION MAGIQUE : FORCER LE TEMPS DE SIMULATION POUR TOUS LES NOEUDS ---
        SetParameter(name='use_sim_time', value=True),

        DeclareLaunchArgument("server_host", default_value="10.111.229.90"),
        DeclareLaunchArgument("server_port", default_value="9900"),
        DeclareLaunchArgument("image_topic", default_value="/rgb_camera/image"),

        # Exports pour les graphiques et le modèle
        SetEnvironmentVariable("PROJECT_MODEL", "turtlebot3_burger_d435i"),
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),

        # Gazebo + RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        ),

        # Robot roaming (Logique d'évitement d'obstacles)
        Node(
            package="turtlebot3_autonomous",
            executable="obstacle_avoider",
            name="movement_logic",
            output="screen",
        ),

        # --- LES NOEUDS IA (PERSON_DETECTOR ET GESTURE) SONT DÉSACTIVÉS ---
        # Pour éviter les timeouts réseaux et les erreurs NumPy sur cette machine.

        # Supervisor (Le cerveau qui gère les états du robot)
        Node(
            package="robot_supervisor",
            executable="supervisor",
            name="brain",
            output="screen",
        ),
    ])