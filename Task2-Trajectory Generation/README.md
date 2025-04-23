## Demo Videos:
All demo videos are documented here: https://1drv.ms/f/s!ApjXPmewijhuzIYywlM5ao0sXWdFaQ?e=1Wud02

## Results:
The task report is documented in Task2-Trajectory Generation.pdf

# Franka Robot Trajectory Generation

This repository contains tools for generating and executing trajectories for the Franka Emika Panda robot arm in both simulation and real hardware environments.

## Prerequisites

- Ubuntu (18.04 or 20.04 recommended)
- ROS Noetic or ROS Melodic
- Docker and Docker Compose

## Setup Instructions

### Docker Setup

1. Modify the `.env` file:
   ```bash
   # Replace <username> with your actual username
   USERNAME=<username>
   ROS_WS=/home/<username>/ros_ws
   ```

2. Build the Docker image:
   ```bash
   ./setup_docker.bash
   ```

3. Start the Docker container:
   ```bash
   docker compose up
   ```

4. Access the running container:
   ```bash
   docker compose exec -it gazebo_franka_ros1-ros1_node-1 bash
   ```

5. Initialize your workspace (first time only):
   ```bash
   catkin_make
   ```

## Running Simulations

### Basic Robot Simulation
```bash
roslaunch franka_gazebo panda.launch x:=-0.5 \
    controller:=cartesian_impedance_example_controller \
    use_gripper:=false \
    rviz:=true
```

### Pick-and-Place Simulation
```bash
roslaunch franka_gazebo panda.launch x:=-0.5 \
    world:=$(rospack find demo_pkg)/world/pick_place.sdf \
    controller:=cartesian_impedance_example_controller \
    rviz:=true
```

### Polishing Task Simulation
```bash
roslaunch franka_gazebo panda.launch \
    world:=$(rospack find demo_pkg)/world/polish.sdf \
    controller:=cartesian_impedance_example_controller \
    use_gripper:=false \
    rviz:=false \
    interactive_marker:=false
```

## Running Your Trajectory Generator

After the simulation is running:

1. Build your package:
   ```bash
   catkin_make
   ```

2. Run your trajectory generator node:
   ```bash
   rosrun trajectory_gen trajectory_gen_node
   ```

3. Monitor robot pose commands:
   ```bash
   rostopic echo /cartesian_impedance_example_controller/equilibrium_pose
   ```

4. Check publishers and subscribers:
   ```bash
   rostopic info /cartesian_impedance_example_controller/equilibrium_pose
   ```

## Running on Real Hardware

To run the controller on the real Panda robot:

```bash
roslaunch trajectory_gen cartesian_impedance_example_controller.launch \
    robot:=panda \
    robot_ip:=192.168.3.8
```

## Gripper Control

Send a grasp command:

```bash
rostopic pub /franka_gripper/grasp/goal franka_gripper/GraspActionGoal "header:
  seq: 1
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  width: 0.0
  epsilon:
    inner: 0.0
    outer: 0.0
  speed: 0.5
  force: 1.0"
```

## Data Recording

Record important topics for later analysis:

```bash
rosbag record rosout /franka_state_controller/F_ext /cartesian_impedance_example_controller/equilibrium_pose
```
