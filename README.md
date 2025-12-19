# MyBot - ROS 2 Mobile Robot Simulation Package

## Overview

**MyBot** is a ROS 2 package that provides a complete simulation environment for a mobile differential-drive robot equipped with sensors. The package includes robot description files, Gazebo world configurations, and launch scripts for simulation and visualization.

## Package Features

### Robot Configuration
- **Differential Drive Robot**: Two-wheel drive system with caster wheel
- **Sensors**:
  - LiDAR/Laser Scanner (RPLiDAR simulation)
  - Depth Camera with 3D point cloud
  - RGB Camera
  - Odometry from wheel encoders
- **Joints**: Fully articulated with visual and collision meshes

### Simulation Environment
- **Gazebo Integration**: Uses Gazebo Ignition (ros_gz_sim) for physics simulation
- **Multiple World Scenarios**:
  - `tugbot_warehouse.sdf` - Warehouse environment (default)
  - `tugbot.world` - Open environment
  - `obstacles.world` - Obstacle course
- **ROS 2 Bridging**: ros_gz_bridge for topic communication

### Visualization
- **RViz2 Integration**: Real-time visualization of robot state and sensor data
- **Custom Configuration**: Pre-configured RViz setup for robot control and mapping

## Project Structure

```
mybot/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # ROS 2 package metadata
├── LICENSE.md                  # Apache 2.0 License
│
├── description/                # URDF/Xacro robot description
│   ├── robot.urdf.xacro       # Main robot description (includes all components)
│   ├── robot_core.xacro       # Base chassis, wheels, caster wheel
│   ├── lidar.xacro            # LiDAR/laser scanner configuration
│   ├── camera.xacro           # RGB camera definition
│   ├── depth_camera.xacro     # Depth camera with point cloud
│   ├── gazebo_control.xacro   # Gazebo plugins and differential drive controller
│   └── inertial_macros.xacro  # Helper macros for inertia calculations
│
├── launch/                     # Launch files for different scenarios
│   ├── rsp.launch.py          # Robot State Publisher (processes xacro files)
│   ├── launch_sim.launch.py   # Complete simulation with Gazebo and RViz
│   ├── slam.launch.py         # SLAM Toolbox integration for mapping
│   └── rplidar.launch.py      # RPLiDAR-specific configuration
│
├── config/                     # Configuration files
│   ├── mapper_params_online_async.yaml  # SLAM Toolbox configuration
│   ├── drive_robot.rviz                 # RViz visualization settings
│   └── empty.yaml              # Empty world configuration
│
├── models/                     # Gazebo model files
│   └── tugbot/                # Robot model directory
│
└── worlds/                     # Gazebo world/environment files
    ├── tugbot_warehouse.sdf   # Warehouse environment
    ├── tugbot.world           # Simple environment
    └── tugbot_warehouse.sdf   # Alternative format
```

## Dependencies

### Build Dependencies
- `ament_cmake` - ROS 2 build system
- `xacro` - URDF macro processor

### Runtime Dependencies
- `ros_gz_sim` - Gazebo Ignition simulator
- `ros_gz_bridge` - ROS-Gazebo message bridge
- `robot_state_publisher` - Publishes robot transform tree
- `joint_state_publisher` - Publishes joint states
- `rviz2` - ROS 2 visualization tool
- `slam_toolbox` - SLAM implementation for mapping

## Installation & Setup

### Prerequisites
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select mybot
source install/setup.bash
```

## Usage

### 1. Launch Complete Simulation
Starts Gazebo, spawns robot, bridges topics, and opens RViz:
```bash
ros2 launch mybot launch_sim.launch.py
```

**Optional arguments:**
```bash
ros2 launch mybot launch_sim.launch.py world:=/path/to/custom.sdf
```

### 2. Launch SLAM Mapping
Starts SLAM Toolbox for autonomous mapping and localization:
```bash
ros2 launch mybot slam.launch.py
```

This requires `launch_sim.launch.py` running in another terminal.

### 3. Robot State Publisher Only
Processes robot description and publishes transforms (no simulation):
```bash
ros2 launch mybot rsp.launch.py use_sim_time:=false
```

### 4. RPLiDAR-Specific Setup
Launches simulation with RPLiDAR configuration:
```bash
ros2 launch mybot rplidar.launch.py
```

## Topic Communication

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/clock` | `rosgraph_msgs/Clock` | Simulation clock |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR point data |
| `/odom` | `nav_msgs/Odometry` | Odometry from wheel encoders |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |
| `/camera/image` | `sensor_msgs/Image` | RGB camera image |
| `/camera/depth_image` | `sensor_msgs/Image` | Depth camera image |
| `/camera/points` | `sensor_msgs/PointCloud2` | 3D point cloud |
| `/camera_info` | `sensor_msgs/CameraInfo` | Camera calibration info |
| `/joint_states` | `sensor_msgs/JointState` | Joint positions/velocities |

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity commands |

## Robot Description

### Links
- **base_link** - Reference frame
- **chassis** - Robot body (0.3 x 0.3 x 0.15 m)
- **left_wheel** - Left drive wheel
- **right_wheel** - Right drive wheel
- **caster_wheel** - Front caster wheel
- **lidar_link** - LiDAR sensor
- **camera_link** - RGB/depth camera

### Joints
- **chassis_joint** - Fixed joint (base_link to chassis)
- **left_wheel_joint** - Continuous joint (drive)
- **right_wheel_joint** - Continuous joint (drive)
- **caster_wheel_joint** - Continuous joint (free rotation)
- **lidar_joint** - Fixed joint (on chassis)
- **camera_joint** - Fixed joint (on chassis)

## Control

### Velocity Commands
Send velocity commands using:
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.2}}"
```

Where:
- `linear.x` - Forward velocity (m/s)
- `angular.z` - Rotational velocity (rad/s)

## Configuration Files

### SLAM Configuration (`mapper_params_online_async.yaml`)
Configures SLAM Toolbox parameters:
- Async online SLAM mode
- Loop closure detection
- Map update frequency
- Sensor timeout settings

### RViz Configuration (`drive_robot.rviz`)
Pre-configured visualization including:
- Robot model display
- LaserScan visualization
- Camera image display
- Map and odometry visualization
- Coordinate frame display

## Gazebo World Files

### tugbot_warehouse.sdf (Default)
- Warehouse-style environment with walls and obstacles
- Suitable for mapping and navigation tasks

### tugbot.world
- Open environment
- Minimal obstacles for basic testing

### obstacles.world
- Obstacle course for navigation testing

## Extending the Package

### Adding New Sensors
1. Create a new `.xacro` file in `description/`
2. Include it in `robot.urdf.xacro`
3. Configure Gazebo plugin in `gazebo_control.xacro`

### Custom World
1. Create `.sdf` or `.world` file in `worlds/`
2. Reference in launch file:
   ```bash
   ros2 launch mybot launch_sim.launch.py world:=worlds/my_world.sdf
   ```

### Modifying Robot Parameters
- **Size/Weight**: Edit `robot_core.xacro`
- **Wheel Properties**: Adjust radius and width in `robot_core.xacro`
- **Sensor Position**: Modify xyz/rpy in respective `.xacro` files

## Troubleshooting

### Robot doesn't spawn in Gazebo
- Ensure `robot_description` topic is published
- Check RViz `Fixed Frame` is set to `odom` or `base_link`
- Verify Gazebo is running: `gz topic -l`

### No sensor data
- Check bridges in launch file match Gazebo topic names
- Verify sensor plugins are enabled in Gazebo config
- Use `ros2 topic list` to confirm topics exist

### SLAM not working
- Ensure `/scan` topic is being published
- Check SLAM Toolbox parameters in config file
- Verify map is being created: `ros2 topic echo /map`

### RViz crashes
- Reset RViz: `rm ~/.rviz2/default.rviz`
- Check Display configuration file exists
- Verify transforms are published: `ros2 run tf2_tools view_frames`

## License

Apache License 2.0 - See LICENSE.md for details

## Maintainer

**Paul Pritam** (pritampaulwork7@gmail.com)

## Related Packages

- **fsm_bumpgo_cpp** - Bump-and-go finite state machine implementation
- **slam_toolbox** - Autonomous mapping and localization
- **ros_gz_sim** - Gazebo Ignition simulator bridge
