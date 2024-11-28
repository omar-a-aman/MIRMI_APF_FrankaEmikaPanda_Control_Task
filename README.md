# Artifical Potential Field Motion Planning for Franka Emika Panda using Pinocchio Library and Mujoco

This is a simple motion planning implementation of the Artifical Potential Field (APF) Method to move a Franka Emika Panda from start to goal while avoiding a ball.

## Mujoco Setup:
Firstly, we create a file called [start_goal_obstacle_trajectory_viz.xml](models/start_goal_obstacle_trajectory_viz.xml) containing the start point, goal point, obstacle and a marker moving agent with a free joint.
This file can be launched using Mujoco viewer API in Python or using Mujoco directly.
![Screenshot 2024-11-28 205820](https://github.com/user-attachments/assets/26a12c4f-fc0e-4281-9060-5d9ae8b1e3f9)

## Artificial Potential Field:
The artificial potential field algorithm - introduced by Prof. Oussama Khatib in his paper ["Real-Time Obstacle Avoidance for Manipulators and Mobile Robots"](https://journals.sagepub.com/doi/abs/10.1177/027836498600500106) - is used as described in the relevant class in [APF.py](scripts/APF.py). This script loads the XML model, computes a collision-free path from the start to the goal, and simulates it in Mujoco as seen in this video. The attractive force gain (α), the repulsive force gain (β), the influence radius of the obstacles (ρ), the resolution of the solution and the maximum number of iterations are all variables that were adjusted using trial and error to achieve an acceptable suboptimal trajectory.

https://github.com/user-attachments/assets/9925d885-e498-42f8-8b15-8ece79832031

## Franka Emika Panda Control:
Finally, the Franka Emika Panda was imported from the [Mujoco Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main) as shown in this [directory](models/franka_emika_panda). The [script](scripts/Franka_Emika_Panda_Control.py) imports the panda and the start, goal, and obstacle setup, utilizes the motion planner in [APF.py](scripts/APF.py) and controls the joint velocities according to the following control law:
<p align="center">
$V=J^†ke + Nd(q_c - q)$
</p>

<div align="center">
  
| Variable      | Indication                                   |
|:-------------:|:--------------------------------------------:|
|$`V`$          | Joints Velocities                            |
|$`J^†`$        | Moore-Penrose pseudoinverse of the Jacobian  |
|$`k,d`$        | The proportional gains > 0                   |
|$`e`$          | The task space error $`e = x - x_d `$        |
|$`x`$          | Current end effector pose                    |
|$`x_d`$        | Desired end effector pose                    |
|$`N`$          | The Null Space Projector $`N = I - J^†J `$   |
|$`q_c`$        | The joint center $`q_c = 0.5(q_{min}+ q_{max})`$|
|$`q_{min}`$    | Minimum Joint Position                       |
|$`q_{max}`$    | Maximum Joint Position                       |
  
</div>

The implementation in the Python [script](scripts/Franka_Emika_Panda_Control.py) is a derivation of the work of [kevinzakka](https://github.com/kevinzakka) from his [mjctrl repository](https://github.com/kevinzakka/mjctrl).

### Panda Imported in Mujoco:
![Panda](https://github.com/user-attachments/assets/2c4fcf4c-ea9e-4690-83b5-18a743c065e4)
### Panda following the planned trajectory:
https://github.com/user-attachments/assets/ea878d23-2cc2-4f10-958f-4e92fa5afe81


