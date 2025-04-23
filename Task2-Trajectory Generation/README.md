## Demo Videos:
All demo videos are documented here: https://1drv.ms/f/s!ApjXPmewijhuzIYywlM5ao0sXWdFaQ?e=1Wud02

## Results:
The task report is documented in Task2-Trajectory Generation.pdf

Franka Robot Trajectory Generation
This repository contains tools for generating and executing trajectories for the Franka Emika Panda robot arm in both simulation and real hardware environments.
Prerequisites

Ubuntu (18.04 or 20.04 recommended)
ROS Noetic or ROS Melodic
Docker and Docker Compose

Setup Instructions
Docker Setup

Modify the .env file:
bash# Replace <username> with your actual username
USERNAME=<username>
ROS_WS=/home/<username>/ros_ws

Build the Docker image:
bash./setup_docker.bash

Start the Docker container:
bashdocker compose up

Access the running container:
bashdocker compose exec -it gazebo_franka_ros1-ros1_node-1 bash

Initialize your workspace (first time only):
bashcatkin_make


Running Simulations
Basic Robot Simulation
bashroslaunch franka_gazebo panda.launch x:=-0.5 \
    controller:=cartesian_impedance_example_controller \
    use_gripper:=false \
    rviz:=true
Pick-and-Place Simulation
bashroslaunch franka_gazebo panda.launch x:=-0.5 \
    world:=$(rospack find demo_pkg)/world/pick_place.sdf \
    controller:=cartesian_impedance_example_controller \
    rviz:=true
Polishing Task Simulation
bashroslaunch franka_gazebo panda.launch \
    world:=$(rospack find demo_pkg)/world/polish.sdf \
    controller:=cartesian_impedance_example_controller \
    use_gripper:=false \
    rviz:=false \
    interactive_marker:=false
Running Your Trajectory Generator
After the simulation is running:

Build your package:
bashcatkin_make

Run your trajectory generator node:
bashrosrun trajectory_gen trajectory_gen_node

Monitor robot pose commands:
bashrostopic echo /cartesian_impedance_example_controller/equilibrium_pose

Check publishers and subscribers:
bashrostopic info /cartesian_impedance_example_controller/equilibrium_pose


Running on Real Hardware
To run the controller on the real Panda robot:
bashroslaunch trajectory_gen cartesian_impedance_example_controller.launch \
    robot:=panda \
    robot_ip:=192.168.3.8
