# DIA Workspace Setup Guide

This guide helps you set up the DIA workspace on a new computer.

## Prerequisites

- Ubuntu 22.04 (or compatible)
- ROS 2 Humble (or your ROS 2 distribution)
- Git

## Quick Setup

### 1. Clone the Repository

```bash
git clone --recursive https://github.com/YOUR_USERNAME/DIA.git
cd DIA
```

The `--recursive` flag will automatically clone all submodules including:
- `PX4-Autopilot` (branch: dia)
- `px4_msgs`
- `ego-planner-swarm` (branch: ros2_version)
- `Point-LIO` (LIO-SAM alternative for localization)

### 2. Install Dependencies

Run the dependency installation script:

```bash
chmod +x setup_dependencies.sh
./setup_dependencies.sh
```

This script will install:
- **Micro XRCE-DDS Agent**: PX4 communication bridge
- **nvblox**: NVIDIA's ESDF mapping library (requires manual installation)

### 3. Build the Workspace

```bash
cd ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source install/setup.bash
```

### 4. Launch the System

```bash
ros2 launch super_launch system.launch.py
```

## Manual Submodule Update

If you already cloned the repository without `--recursive`:

```bash
cd DIA
git submodule update --init --recursive
```

To update submodules to the latest commits:

```bash
git submodule update --remote
```

## Package Structure

- `sensor_interface`: LiDAR + IMU data provider
- `Point-LIO`: HKU-MARS Point-LIO for localization
- `esdf_mapping`: NVBlox ESDF mapping
- `ego_planning`: EGO-Swarm planner (single UAV)
- `trajectory_control`: B-spline → PX4 TrajectorySetpoint
- `px4_interface`: Micro XRCE-DDS Agent launcher
- `super_launch`: System-wide launch file

## Troubleshooting

### Build Errors

If you encounter build errors, try building packages in order:

```bash
colcon build --packages-select px4_msgs
colcon build --packages-select traj_utils
colcon build
```

### MicroXRCEAgent Not Found

Ensure the agent is installed:

```bash
which MicroXRCEAgent
```

If not found, run the setup script again or install manually.

### nvblox Not Found

Install nvblox following the official guide:
https://nvidia-isaac.github.io/nvblox/

Or use apt (requires CUDA):

```bash
sudo apt-get install ros-$ROS_DISTRO-nvblox-ros
```
