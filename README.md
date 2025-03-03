# ROS2 UGV-UAV Follow Control

A ROS2 project implementing heterogeneous robot collaboration between a ground vehicle (UGV) and an aerial vehicle (UAV).

## Features
- UGV circular motion pattern
- UAV autonomous following
- Model-based control system
- Gazebo/Ignition simulation

## Dependencies
- ROS2 Humble
- ros_gz_sim
- ros_gz_bridge
- Python 3.x

## Installation
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/YOUR_USERNAME/my_suru.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select my_suru
source install/setup.bash
```

## Usage
```bash
# Launch simulation
ros2 launch my_suru sim_launch.py
```

## License
Apache 2.0
