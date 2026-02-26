import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. DÉFINITION DES VARIABLES D'ENVIRONNEMENT
    # Plus besoin de les taper dans le terminal !
    
    # Chemin vers les modèles Gazebo
    fuel_path = os.path.expanduser('~/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models')
    # On récupère l'ancien path s'il existe et on ajoute le nôtre
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        fuel_path = fuel_path + ':' + os.environ['GZ_SIM_RESOURCE_PATH']
        
    env_vars = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', '10'),
        # Tu avais mis les deux, mais CycloneDDS est souvent plus stable pour la vidéo/points
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', fuel_path)
    ]

    # 2. INCLURE LE ROBOT (Terminal 1)
    # ros2 launch robot_supervisor robot_launch.py
    robot_launch_dir = get_package_share_directory('robot_supervisor')
    robot_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_launch_dir, 'launch', 'robot_launch.py') # Vérifie si c'est dans un dossier 'launch'
        )
        # Si robot_launch.py est à la racine du share, retire 'launch' ci-dessus
    )

    # 3. LANCER LA GRILLE (Terminal 2)
    # ros2 run ground_segmentation grid_projection_node ...
    grid_node = Node(
        package='ground_segmentation',
        executable='grid_projection_node',
        name='smart_grid_node',
        output='screen',
        parameters=[
            {'cloud_topic': '/depth_camera/points'},
            {'use_sim_time': True}
        ]
    )

    # 4. LANCER L'ACTIVATEUR (Terminal 3)
    # python3 activator.py (devenu un node via setup.py)
    activator_node = Node(
        package='ground_segmentation',
        executable='activator_node',
        name='gazebo_activator',
        output='screen'
    )

    # On retourne la description complète
    return LaunchDescription(env_vars + [
        robot_simulation,
        grid_node,
        activator_node
    ])