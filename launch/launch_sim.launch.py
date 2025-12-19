import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_mybot = get_package_share_directory('mybot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    
    world_arg = DeclareLaunchArgument(
        'world',
    
        default_value=os.path.join(pkg_mybot, 'worlds', 'tugbot_warehouse.sdf'),
        description='Path to SDF world file'
    )
    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mybot, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items()
    )

   
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'mybot',
        ],
        output='screen'
    )

    rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    parameters=[{'use_sim_time': True}],
    arguments=['-d', '/ros2_ws/src/mybot/config/drive_robot.rviz'] 
)


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',

            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',

            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
             
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
         
            '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
           
            '/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',

            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model'
        ],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        rsp,
        gazebo,
        spawn,
        bridge,
        rviz
    ])