# Artifical Potential Field Motion Planning for Franka Emika Panda using Pinocchio Library and Mujoco

This is a simple motion planning implementation of the Artifical Potential Field (APF) Method to move a Franka Emika Panda from start to goal while avoiding a ball.

## Mujoco Setup:
Firstly, we create a file called ["start_goal_obstacle_trajectory_viz.xml"](models/start_goal_obstacle_trajectory_viz.xml)containing the start point, goal point, obstacle and a marker moving agent with a free joint.
