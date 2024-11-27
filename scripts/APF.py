import numpy as np
import mujoco
import mujoco_viewer
import time

class ArtificialPotentialFields3D:
    def __init__(self, start, goal, obstacle_center, obstacle_radius, 
                 alpha=10.0, beta=100.0, rho_0=4.0, step_size=0.01, max_iters=1000):
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
            if np.linalg.norm(pos - self.goal) < 1e-5:
                print(f"Goal reached in {i} iterations.")
                break
        else:
            print("Maximum iterations reached without convergence.")
        
        return np.array(path)


def create_mujoco_model(start, goal, obstacle_center, obstacle_radius):
    """
    Creates a simple MuJoCo XML model for path visualization.
    """
    mjcf = f"""
    <mujoco model="path_planning">
        <worldbody>
            <!-- Start Marker -->
            <geom type="plane" size="20 20 0.1" rgba="1 1 1 1"/>
            <body name="start_marker" pos="{start[0]} {start[1]} {start[2]}">
                <geom type="sphere" size="0.1" rgba="0 1 0 1" />
            </body>
            <!-- Goal Marker -->
            <body name="goal_marker" pos="{goal[0]} {goal[1]} {goal[2]}">
                <geom type="sphere" size="0.2" rgba="1 0 0 1" />
            </body>
            <!-- Obstacle -->
            <body name="obstacle" pos="{obstacle_center[0]} {obstacle_center[1]} {obstacle_center[2]}">
                <geom type="sphere" size="{obstacle_radius}" rgba="1 0.5 0 0.5" />
            </body>
            <!-- Movable Agent -->
            <body name="agent" pos="{start[0]} {start[1]} {start[2]}">
                <geom type="sphere" size="0.1" rgba="0 0 1 1" />
                <joint name="agent_joint_x" type="slide" axis="1 0 0" />
                <joint name="agent_joint_y" type="slide" axis="0 1 0" />
                <joint name="agent_joint_z" type="slide" axis="0 0 1" />
            </body>
        </worldbody>
    </mujoco>
    """
    return mjcf

def simulate_path(path, model_xml):
    """
    Simulates the agent's path in MuJoCo.
    """
    # Load the model
    model = mujoco.MjModel.from_xml_string(model_xml)
    data = mujoco.MjData(model)
    
    # Create the viewer
    viewer = mujoco_viewer.MujocoViewer(model, data)
    viewer.cam.elevation = -90
    viewer.cam.distance = 40
    viewer.cam.azimuth = 180

    # Simulate the path
    i = 1
    for point in path:
        data.qpos[:3] = point  # Update agent's position
        mujoco.mj_step(model, data)  # Step the simulation
        print(point)
        viewer.render()  # Render the scene
        time.sleep(0.1)
        i+=1
    viewer.close()

# Example usage
if __name__ == "__main__":
    # Define parameters
    start = [0, 0, 0]
    goal = [10, 15, 5]
    obstacle_center = [5, 5, 2]
    obstacle_radius = 2

    # Generate the path using the ArtificialPotentialFields3D
    apf = ArtificialPotentialFields3D(start, goal, obstacle_center, obstacle_radius)
    path = apf.plan()

    # Create the MuJoCo model XML
    mujoco_model = create_mujoco_model(start, goal, obstacle_center, obstacle_radius)

    # Simulate the path in MuJoCo
    simulate_path(path, mujoco_model)