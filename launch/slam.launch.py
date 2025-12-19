import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    pkg_mybot = get_package_share_directory('mybot')
    
    # Path to the YAML file 
    slam_config_file = os.path.join(pkg_mybot, 'config', 'mapper_params_online_async.yaml')

    
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_config_file
        }.items()
    )

    return LaunchDescription([
        slam_toolbox
    ])