import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_mybot = get_package_share_directory("mybot")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_mybot, 'worlds', 'tugbot.world'),
        description='Path to the world file to load into Gazebo'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

 
    world_path = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mybot, "launch", "rsp.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "verbose": "true", 
            "world": world_path
        }.items()
    )

   
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "mybot",
            "-topic", "robot_description"
            
        ],
    )

    return LaunchDescription([
        world_arg,
        sim_time_arg,
        rsp,
        gazebo,
        spawn
    ])