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

    # Arguments
    # Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        # UPDATE THIS LINE TO POINT TO THE NEW FILE
        default_value=os.path.join(pkg_mybot, 'worlds', 'tugbot_warehouse.sdf'),
        description='Path to SDF world file'
    )
    # 1. Robot State Publisher (Parses URDF)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mybot, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Start Ignition Gazebo (gz_sim)
    # "-r" runs the simulation immediately on startup
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items()
    )

    # 3. Spawn Robot
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', # Reads URDF from the RSP topic
            '-name', 'mybot',
        ],
        output='screen'
    )

    # 4. Bridge (Links Ignition Topics to ROS Topics)
    # Bridges: Clock (Ign->ROS) and Cmd_vel (ROS->Ign)
    # 4. Bridge (Links Ignition Topics to ROS Topics)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # SYSTEM ---------------------------------------------------------
            # Clock (Ign -> ROS) - Essential for "use_sim_time"
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',

            # DIFF DRIVE PLUGIN ----------------------------------------------
            # Cmd_vel (ROS -> Ign) - Drive the robot
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            
            # Odometry (Ign -> ROS) - Robot position feedback
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            
            # TF (Ign -> ROS) - Odom to Base_link transform
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',

            # LIDAR ----------------------------------------------------------
            # Scan (Ign -> ROS) - Laser data
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            
             # RGB Image
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            # Depth Image
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            # Point Cloud
            '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            # Camera Info
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
        bridge
    ])