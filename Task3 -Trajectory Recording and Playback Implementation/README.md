# Franka Robot Trajectory Generation and Analysis

This repository contains implementation and analysis tools for generating, recording, and replaying trajectories on Franka Emika Panda robots.

## Overview

This project implements a complete trajectory handling system for robotic arms, featuring:

- Hand-guided trajectory recording at 100Hz resolution
- Trajectory replay using impedance control
- Comprehensive analysis of trajectory accuracy and performance
- Visualization tools for comparing original and replayed paths

## Key Features

- **High-Resolution Data Collection**: Captures robot state every 0.01 seconds with end-effector pose information
- **JSON-based Trajectory Storage**: Records complete trajectories for later analysis and replay
- **Impedance Control System**: Implements torque-based replay of recorded trajectories
- **Detailed Performance Analysis**: Evaluates differences between hand-guided and replayed trajectories

## Implementation Details

### Recording System
- Captures data at 100Hz (every 0.01 seconds)
- Maximum recording duration of 5 seconds per trajectory
- Stores end-effector pose using 4×4 homogeneous transformation matrices
- Outputs trajectory data to JSON format

### Replay System
- Uses impedance control methodology
- Loads positions from JSON files
- Converts recorded trajectory data into joint torque commands
- Samples at 1000Hz during replay execution

## Analysis Tools

The repository includes Python-based visualization tools that generate comparative plots showing:
- Original hand-guided trajectory paths
- Replayed trajectory execution
- Point-by-point difference analysis between original and replayed paths

## Technical Insights

Our analysis examines several factors affecting trajectory replay accuracy:
- **Sampling Rate Differences**: Recording (100Hz) vs. replay (1000Hz)
- **Temporal Misalignment**: Challenges in direct point comparison
- **Interpolation Effects**: Estimation errors between original data points
- **Physical Limitations**: Robot arm speed and acceleration constraints
- **Control Parameters**: Impact of impedance stiffness settings
- **Sensor Noise**: Measurement errors and accumulated deviations
- **Control Loop Timing**: Processing delays and synchronization challenges

## Running the Code

1. Build the project:
   ```
   cd build
   cmake .. && make
   ```

2. Run trajectory recording:
   ```
   ./02_trajectory_recording
   ```

3. Run trajectory replay:
   ```
   ./04_replay_trajectory
   ```

4. Analyze results:
   ```
   jupyter notebook 05_generate_plot_trajectory.ipynb
   ```

## Requirements

- Physical Franka Emika Panda robot
- Network connection to robot control system
- C++17 compatible compiler
- Python with matplotlib, numpy, and pandas for analysis

## Documentation

See `IMPLAssignment3.pdf` for detailed information about the methodology and analysis results.
