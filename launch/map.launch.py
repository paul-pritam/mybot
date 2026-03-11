import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_mybot = get_package_share_directory('mybot')

    # Define file paths
    map_file = os.path.join(pkg_mybot, 'maps', 'my_map.yaml')
    amcl_file = os.path.join(pkg_mybot, 'config', 'amcl.yaml')

    # Create the Map Server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file},
                    {'use_sim_time': True}]
    )

    # Create the AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',        
        output='screen',
        parameters=[amcl_file, 
                    {'use_sim_time': True}]
    )   

    # Create the Lifecycle Manager node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager', 
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    return LaunchDescription([
        map_server_node,
        amcl_node,
        lifecycle_manager_node
    ])