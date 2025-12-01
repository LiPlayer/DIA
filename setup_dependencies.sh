#!/bin/bash

# DIA Workspace Dependency Installation Script
# This script installs all required dependencies for the DIA ROS2 workspace

set -e  # Exit on error

echo "=========================================="
echo "DIA Workspace Dependency Setup"
echo "=========================================="

# Check if running on Ubuntu/Debian
if ! command -v apt-get &> /dev/null; then
    echo "Error: This script requires apt-get (Ubuntu/Debian)"
    exit 1
fi

# Install Micro XRCE-DDS Agent
echo ""
echo "Installing Micro XRCE-DDS Agent..."
if command -v MicroXRCEAgent &> /dev/null; then
    echo "MicroXRCEAgent already installed at $(which MicroXRCEAgent)"
else
    echo "Cloning Micro-XRCE-DDS-Agent..."
    cd /tmp
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir -p build && cd build
    cmake ..
    make
    sudo make install
    echo "MicroXRCEAgent installed successfully"
    cd /tmp
    rm -rf Micro-XRCE-DDS-Agent
fi

# Install nvblox
echo ""
echo "Checking for nvblox..."
if ros2 pkg list | grep -q nvblox; then
    echo "nvblox already installed"
else
    echo "nvblox not found. Please install manually following:"
    echo "https://nvidia-isaac.github.io/nvblox/"
    echo ""
    echo "Quick install (requires CUDA):"
    echo "  sudo apt-get install ros-\$ROS_DISTRO-nvblox-ros"
    echo ""
fi

# Initialize git submodules
echo ""
echo "Initializing git submodules..."
cd /home/li/DIA
if [ -f .gitmodules ]; then
    git submodule update --init --recursive
    echo "Git submodules initialized"
else
    echo "No .gitmodules file found. Please run the submodule setup script."
fi

echo ""
echo "=========================================="
echo "Dependency setup complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Source your ROS2 workspace: source /opt/ros/\$ROS_DISTRO/setup.bash"
echo "2. Build the workspace: cd ~/DIA/ros2_ws && colcon build"
echo "3. Source the workspace: source ~/DIA/ros2_ws/install/setup.bash"
