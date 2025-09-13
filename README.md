# Particle Filter for Robot Localization (ROS 2)

This repository contains an implementation of Particle Filter-based localization for a mobile robot in simulation. The system integrates with Gazebo, RViz, and a navigation stack to demonstrate robot localization and path planning.


## 🚀 Setup Instructions
### 1. Clone into ROS 2 Workspace

Navigate to the src folder of your ROS 2 workspace and pull this repository directly (not inside a repo-named folder):

`cd ~/ros2_ws/src
git clone --depth 1 <repo_link> .`


### 2. Build the Workspace

`cd ~/ros2_ws
colcon build`

### 3. Source the Workspace

`source install/setup.bash`

## 🖥️ Running the System

You will need 4 terminals. Run each step in a separate terminal:

### Terminal 1: Launch Gazebo Simulation

`ros2 launch robile_gazebo gazebo_4_wheel.launch.py`

In RViz:

Go to File → Open Config

Load: robile_slam/config/localization_config.rviz

If the robot spawns near a wall or corner, use the teleop keyboard to move it toward the center of the arena.

### Terminal 2: Run Particle Filter Node

`ros2 run robile_slam particle_filter_node`

### Terminal 3: Start Navigation

`ros2 run robile_slam navigation.launch.py`


### Terminal 4: Launch Map Server

`ros2 launch robile_slam map_server.launch.py`

### ✅ Verification

Check the following logs to confirm everything is working:

## Particle Filter Terminal → should say:

`publishing amcl_pose`

## Navigation Terminal → should show:

`[global_planner] found path to goal`

# ⚠️ Common Issues

## Map not received error

- Can occur randomly.

- Ensure the launch order is followed exactly.

## Particle Filter not working

- Sometimes fails randomly.

- Restart from Step 2 (re-run particle filter node).


# 🎯 Using RViz for Localization & Navigation

In RViz, select 2D Pose Estimate and drag the arrow to approximately match the robot’s actual position in the arena.

Set a goal using 2D Goal Pose.

You should see a planned path, and the robot should start moving toward the goal.
