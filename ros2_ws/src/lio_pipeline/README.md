# LIO Pipeline

This directory contains different LIO (LiDAR-Inertial Odometry) implementations for testing and comparison.

## Available Implementations

### Point-LIO
- **Source**: [HKU-MARS Point-LIO](https://github.com/hku-mars/Point-LIO)
- **Type**: Point-based LIO
- **Status**: Integrated as git submodule
- **Location**: `Point-LIO/`

### FAST-LIO (Planned)
- **Source**: TBD
- **Type**: Fast LIO-SAM
- **Status**: To be added
- **Location**: `FAST-LIO/` (future)

## Usage

Each LIO implementation is maintained as a separate git submodule under this directory. This allows for easy testing and comparison of different algorithms.

To switch between implementations, update the launch files in `super_launch` to point to the desired LIO package.
