## Overview
URFI_project simulates a six‑wheeled rover in Ignition Fortress Gazebo using ROS 2 Humble on Ubuntu 22.04, providing modular packages for robot description, control, simulation integration, and autonomous navigation&#8203;.

## Video Demonstration
[![URFI Rover Demo](https://img.youtube.com/vi/qBmY7B5HzhM/0.jpg)](https://youtu.be/qBmY7B5HzhM)  
Click to watch the rover navigating a custom world in Ignition Fortress&#8203;.

## Prerequisites
- **Ubuntu 22.04 LTS** – Tier 1 platform for ROS 2 Humble Hawksbill&#8203;
- **ROS 2 Humble** – Install desktop packages (`ros-humble-desktop`) via apt per official guide&#8203; 
- **Ignition Fortress** – Recommended simulation engine paired with ROS 2 Humble; install via `sudo apt install ignition-fortress`&#8203;
- **colcon** – Build tool for ROS 2 workspaces; installed with `ros-humble-colcon-common-extensions`  
- **rosdep** – Dependency resolver for ROS distributions; initialize and update before building&#8203;:

## Installation
```bash
# Clone repository and prepare workspace
git clone https://github.com/alexoberco/urfi_project.git
cd urfi_project
mkdir -p ros2_ws/src
mv * ros2_ws/src/
cd ros2_ws

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -y

# Build and source
colcon build --symlink-install
source install/setup.bash
```
Use `colcon build` from your workspace root to compile ROS 2 packages.

## Workspace Structure
```
ros2_ws/
├── src/
│   ├── starter/             # Core nodes, launch scripts & tests
│   ├── urfi_description/    # URDF & SDF models, meshes & RViz configs
│   ├── urfi_control/        # ros2_control controller configs
│   ├── urfi_gz_ignition/    # Ignition Fortress launch integrations
│   └── urfi_navigation/     # Nav2 stack configs, maps & demos
├── install/
├── build/
└── log/
```
Following standard ROS 2 workspace conventions, all packages reside under `src/`&#8203;.

## Package Details
### starter  
Includes template C++/Python nodes, launch files, and basic tests to verify workspace integrity.

### urfi_description  
- **urdf/**: XACRO definitions of the six‑wheeled rover  
- **meshes/**: STL/DAE files for visual and collision models  
- **worlds/**: SDF files defining custom Ignition worlds  
- **rviz/**: RViz configuration for real-time visualization  

### urfi_control  
Configures `ros2_control` controllers (PID gains, joint limits) for wheel and steering interfaces&#8203;.

### urfi_gz_ignition  
Launch files that:  
1. Start Ignition Fortress with a custom world  
2. Spawn rover via `ros2 run ros_gz_bridge parameter_bridge`  
3. Bridge ROS 2 topics and services with Ignition&#8203;

### urfi_navigation  
Sets up Nav2 for autonomous navigation:  
- **maps/**: Predefined occupancy grids  
- **config/**: `nav2_params.yaml` and costmap settings  
- **launch/**: Demo launch to test autonomous path planning and recovery  

## Usage Examples
- **Simulation Launch**  
  `ros2 launch urfi_gz_ignition simulation_launch.py`  
- **Teleoperation**  
  `ros2 launch starter teleop_launch.py`  
- **Navigation Demo**  
  `ros2 launch urfi_navigation nav2_demo_launch.py`  
- **RViz Visualization**  
  `ros2 launch urfi_description view_rviz_launch.py`  

## Contributing
Contributions are welcome! Please fork the repo, create a feature branch, and submit a pull request against `main`. Ensure new packages include appropriate launch demos and tests&#8203;.

## License
This project is licensed under the **MIT License**, providing a permissive open-source framework while ensuring proper attribution to authors&#8203;.
