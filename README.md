# Artifical Potential Field Motion Planning for Franka Emika Panda using Pinocchio Library and Mujoco

This is a simple motion planning implementation of the Artifical Potential Field (APF) Method to move a Franka Emika Panda from start to goal while avoiding a ball.

## Mujoco Setup:
Firstly, we create a file called ["start_goal_obstacle_trajectory_viz.xml"](models/start_goal_obstacle_trajectory_viz.xml)containing the start point, goal point, obstacle and a marker moving agent with a free joint.
This file can be launched using Mujoco viewer API in Python or using Mujoco directly.
![Screenshot 2024-11-28 205820](https://github.com/user-attachments/assets/26a12c4f-fc0e-4281-9060-5d9ae8b1e3f9)

## Artificial Potential Field:
The artificial potential field algorithm is used as described in the relevant class in [APF.py](scripts/APF.py). This script loads the XML model, computes a collision-free path from the start to the goal, and simulates it in Mujoco as seen in this video. The attractive force gain, the repulsive force gain, the influence radius of the obstacles, the resolution of the solution and the maximum number of iterations are all variables that were adjusted using trial and error to achieve an acceptable suboptimal trajectory.

https://github.com/user-attachments/assets/9925d885-e498-42f8-8b15-8ece79832031

