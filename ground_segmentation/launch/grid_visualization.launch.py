from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('grid_size', default_value='5.0'),
        DeclareLaunchArgument('grid_n', default_value='20'),
        
        # Nœud de grille interactive
        Node(
            package='ground_segmentation',
            executable='interactive_grid_node',
            name='interactive_grid',
            output='screen',
            parameters=[{
                'grid_size_m': LaunchConfiguration('grid_size'),
                'grid_resolution_n': LaunchConfiguration('grid_n'),
                'grid_height': 0.0,
                'frame_id': 'odom'  # Frame du TurtleBot3
            }]
        ),
        
        # Nœud de projection (optionnel pour commencer)
        # Node(
        #     package='ground_segmentation',
        #     executable='grid_projection_node',
        #     name='grid_projection',
        #     output='screen',
        #     parameters=[{
        #         'grid_size_m': LaunchConfiguration('grid_size'),
        #         'grid_resolution_n': LaunchConfiguration('grid_n'),
        #     }]
        # ),
    ])
