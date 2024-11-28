import numpy as np
import mujoco
import mujoco.viewer
import time
from os.path import dirname, abspath


model_path = dirname(dirname(abspath(__file__)))
urdf_filename = "start_goal_obstacle_trajectory_viz.xml"
urdf_model_path = model_path + "/models/" + urdf_filename

# Load the model and data.
model = mujoco.MjModel.from_xml_path(urdf_model_path)
data = mujoco.MjData(model)

class ArtificialPotentialFields3D:
    def __init__(self, start, goal, obstacle_center, obstacle_radius, 
                 alpha=10.0, beta=100.0, rho_0=4.0, step_size=0.01, max_iters=10000,resolution = 1e-5):
        """
        Initialize the APF planner.
        :param start: Starting position as [x, y, z].
        :param goal: Goal position as [x, y, z].
        :param obstacle_center: Center of the spherical obstacle [x, y, z].
        :param obstacle_radius: Radius of the spherical obstacle.
        :param alpha: Gain for attractive potential.
        :param beta: Gain for repulsive potential.
        :param rho_0: Influence radius of the obstacle.
        :param step_size: Step size for gradient descent.
        :param max_iters: Maximum number of iterations.
        """
        self.start = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)
        self.obstacle_center = np.array(obstacle_center, dtype=float)
        self.obstacle_radius = obstacle_radius
        self.alpha = alpha
        self.beta = beta
        self.rho_0 = rho_0
        self.step_size = step_size
        self.max_iters = max_iters
        self.resolutin = resolution

    def attractive_potential(self, pos):
        """Compute attractive potential and its gradient."""
        diff = pos - self.goal
        U_att = 0.5 * self.alpha * np.linalg.norm(diff) ** 2
        grad_U_att = self.alpha * diff
        return U_att, grad_U_att
    
    def repulsive_potential(self, pos):
        """Compute repulsive potential and its gradient."""
        diff = pos - self.obstacle_center
        rho = np.linalg.norm(diff) - self.obstacle_radius
        if rho < 0:  # Inside the obstacle
            U_rep = float('inf')
            grad_U_rep = np.zeros_like(pos)
        elif rho < self.rho_0:  # Within influence radius
            U_rep = 0.5 * self.beta * ((1 / rho) - (1 / self.rho_0)) ** 2
            grad_U_rep = self.beta * ((1 / rho) - (1 / self.rho_0)) * (1 / rho**2) * (diff / np.linalg.norm(diff))
        else:  # Outside influence radius
            U_rep = 0
            grad_U_rep = np.zeros_like(pos)
        return U_rep, grad_U_rep
    
    def plan(self):
        """Plan the path from start to goal avoiding the obstacle."""
        pos = np.array(self.start, dtype=float)
        path = [pos.copy()]
        
        for i in range(self.max_iters):
            # Compute potentials and gradients
            _, grad_U_att = self.attractive_potential(pos)
            _, grad_U_rep = self.repulsive_potential(pos)
            
            # Combine gradients
            grad_U = -grad_U_att + grad_U_rep
            
            # Update position
            pos += self.step_size * grad_U
            path.append(pos.copy())
            
            # Check for convergence
            if np.linalg.norm(pos - self.goal) < self.resolutin:
                print(f"Goal reached in {i} iterations.")
                break
        else:
            print("Maximum iterations reached without convergence.")
        
        return np.array(path)

def simulate_path(path):
    """
    Simulates the agent's path in MuJoCo.
    """
    # Create the viewer
    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        show_left_ui=False,
        show_right_ui=False,
    ) as viewer:
        # Reset the free camera.
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Enable site frame visualization.
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE

        # Simulate the path
        for point in path:
            data.qpos[0] = point [0] # Update agent's position
            data.qpos[1] = point [1]
            data.qpos[2] = point [2]
            print("point is ", point)
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.05)

    
    
# Example usage
if __name__ == "__main__":
    # Define parameters
    start = [0.55, 0.0, 0.6]
    goal = [0.3, -0.1, 0.3]
    obstacle_center = [0.4, 0, 0.4]
    obstacle_radius = 0.08

    # Generate the path using the ArtificialPotentialFields3D
    apf = ArtificialPotentialFields3D(start, goal, obstacle_center, obstacle_radius
                                           ,alpha=1, beta=0.01,rho_0=0.09,step_size=0.01,resolution=1e-5)
    
    path = apf.plan()
    
    # Simulate the path in MuJoCo
    simulate_path(path)