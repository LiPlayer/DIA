#!/bin/bash

# DIA Workspace Dependency Installation Script
# This script installs all required dependencies for the DIA ROS2 workspace

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${SCRIPT_DIR}/ros2_ws"

echo "=========================================="
echo "DIA Workspace Dependency Setup"
echo "=========================================="

# Check if running on Ubuntu/Debian
if ! command -v apt-get &> /dev/null; then
    echo "Error: This script requires apt-get (Ubuntu/Debian)"
    exit 1
fi

# Ensure ROS 2 environment is available for python path / pkg configs
TARGET_ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${TARGET_ROS_DISTRO}/setup.bash"
if [ -z "${ROS_DISTRO:-}" ]; then
    echo "ROS_DISTRO is not set. Attempting to source ${ROS_SETUP} ..."
else
    echo "ROS_DISTRO is set to ${ROS_DISTRO}. Using ${ROS_SETUP} ..."
fi
if [ -f "${ROS_SETUP}" ]; then
    # shellcheck source=/dev/null
    source "${ROS_SETUP}"
else
    echo "Error: ${ROS_SETUP} not found. Please install or source your ROS 2 distribution."
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

# Install Point-LIO and LIO-SAM dependencies
echo ""
echo "Installing Point-LIO and LIO-SAM dependencies..."
APT_DEPENDENCIES=(
    "python3-ament-package"
    "python3-colcon-common-extensions"
    "python3-rosdep"
    "ros-$ROS_DISTRO-pcl-ros"
    "ros-$ROS_DISTRO-pcl-conversions"
    "ros-$ROS_DISTRO-visualization-msgs"
    "ros-$ROS_DISTRO-perception-pcl"
    "ros-$ROS_DISTRO-pcl-msgs"
    "ros-$ROS_DISTRO-vision-opencv"
    "ros-$ROS_DISTRO-cv-bridge"
    "ros-$ROS_DISTRO-xacro"
    "libpcl-dev"
    "libpcl-all-dev"
    "libgtsam-dev"
    "libeigen3-dev"
    "libomp-dev"
    "libopencv-dev"
)

for dep in "${APT_DEPENDENCIES[@]}"; do
    if dpkg -l | grep -q "$dep"; then
        echo "$dep is already installed."
    else
        echo "Installing $dep..."
        sudo apt-get install -y "$dep"
    fi
done

# Resolve remaining rosdep keys for the workspace
echo ""
if command -v rosdep &> /dev/null; then
    if [ -d "${ROS2_WS}/src" ]; then
        echo "Running rosdep install for ${ROS2_WS}/src ..."
        if ! rosdep update; then
            echo "Warning: rosdep update failed, continuing with existing database."
        fi
        if ! rosdep install --from-paths "${ROS2_WS}/src" --ignore-src -r -y; then
            echo "Warning: rosdep install reported missing keys. Please review the output above."
        fi
    else
        echo "Workspace source folder ${ROS2_WS}/src not found; skipping rosdep install."
    fi
else
    echo "rosdep not found; skipping rosdep install."
fi

# Initialize git submodules
echo ""
echo "Initializing git submodules..."
cd "${SCRIPT_DIR}"
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
echo "2. Build the workspace: cd \"${ROS2_WS}\" && colcon build"
echo "3. Source the workspace: source \"${ROS2_WS}/install/setup.bash\""
