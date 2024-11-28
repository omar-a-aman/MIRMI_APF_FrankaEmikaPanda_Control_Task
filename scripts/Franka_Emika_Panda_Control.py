import mujoco
import mujoco.viewer
import numpy as np
import time
from APF import ArtificialPotentialFields3D
from os.path import dirname, abspath

# Integration timestep in seconds. This corresponds to the amount of time the joint
# velocities will be integrated for to obtain the desired joint positions.
integration_dt: float = 0.1

# Damping term for the pseudoinverse. This is used to prevent joint velocities from
# becoming too large when the Jacobian is close to singular.
damping: float = 1e-4

# Gains for the twist computation. These should be between 0 and 1. 0 means no
# movement, 1 means move the end-effector to the target in one integration step.
Kpos: float = 0.95
Kori: float = 0.95

# Whether to enable gravity compensation.
gravity_compensation: bool = True

# Simulation timestep in seconds.
dt: float = 0.002

# Nullspace P gain.
Kn = np.asarray([10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0])

# Maximum allowable joint velocity in rad/s.
max_angvel = 0.785


def main() -> None:
    assert mujoco.__version__ >= "3.1.0", "Please upgrade to mujoco 3.1.0 or later."

    model_path = dirname(dirname(abspath(__file__)))
    urdf_filename = "scene.xml"
    urdf_model_path = model_path + "/models/franka_emika_panda/" + urdf_filename

    # Load the model and data.
    model = mujoco.MjModel.from_xml_path(urdf_model_path)
    data = mujoco.MjData(model)

    # Enable gravity compensation. Set to 0.0 to disable.
    model.body_gravcomp[:] = float(gravity_compensation)
    model.opt.timestep = dt

    # End-effector site we wish to control.
    site_name = "attachment_site"
    site_id = model.site(site_name).id
    
    start = [0.55, 0.0, 0.6]
    goal = [0.3, -0.1, 0.3]
    obstacle_center = [0.4, 0, 0.4]
    obstacle_radius = 0.08

    apf = ArtificialPotentialFields3D(start, goal, obstacle_center, obstacle_radius
                                           ,alpha=1, beta=0.01,rho_0=0.09,step_size=0.01,resolution=1e-5)
    path = apf.plan()
    # Get the dof and actuator ids for the joints we wish to control. These are copied
    # from the XML file. Feel free to comment out some joints to see the effect on
    # the controller.
    joint_names = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint7",
    ]
    dof_ids = np.array([model.joint(name).id for name in joint_names])
    actuator_ids = np.array([model.actuator(name).id for name in joint_names])

    # Initial joint configuration saved as a keyframe in the XML file.
    key_name = "home"
    key_id = model.key(key_name).id
    q0 = model.key(key_name).qpos

    # Mocap body we will control with our mouse.
    #mocap_name = "target"
    #mocap_id = model.body(mocap_name).mocapid[0]

    # Pre-allocate numpy arrays.
    jac = np.zeros((6, model.nv))
    diag = damping * np.eye(6)
    eye = np.eye(model.nv)
    twist = np.zeros(6)
    site_quat = np.zeros(4)
    site_quat_conj = np.zeros(4)
    error_quat = np.zeros(4)

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        show_left_ui=False,
        show_right_ui=False,
    ) as viewer:
        # Reset the simulation.
        mujoco.mj_resetDataKeyframe(model, data, key_id)

        # Reset the free camera.
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Enable site frame visualization.
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
        
        for point in path:
            step_start = time.time()
            print("current pose:",data.site(site_id).xpos)
            # Spatial velocity (aka twist).
            dx = point - data.site(site_id).xpos
            twist[:3] = Kpos * dx / integration_dt
            mujoco.mju_mat2Quat(site_quat, data.site(site_id).xmat)
            mujoco.mju_negQuat(site_quat_conj, site_quat)
            mujoco.mju_mulQuat(error_quat, site_quat_conj, site_quat_conj)
            mujoco.mju_quat2Vel(twist[3:], error_quat, 1.0)
            twist[3:] *= Kori / integration_dt

            # Jacobian.
            mujoco.mj_jacSite(model, data, jac[:3], jac[3:], site_id)

            # Damped least squares.
            dq = jac.T @ np.linalg.solve(jac @ jac.T + diag, twist)

            # Nullspace control biasing joint velocities towards the home configuration.
            dq += (eye - np.linalg.pinv(jac) @ jac) @ (Kn * (q0 - data.qpos[dof_ids]))

            # Clamp maximum joint velocity.
            '''dq_abs_max = np.abs(dq).max()
               if dq_abs_max > max_angvel:
               dq *= max_angvel / dq_abs_max'''

            # Integrate joint velocities to obtain joint positions.
            q = data.qpos.copy()  # Note the copy here is important.
            mujoco.mj_integratePos(model, q, dq, integration_dt)
            np.clip(q, *model.jnt_range.T, out=q)

            # Set the control signal and step the simulation.
            data.ctrl[actuator_ids] = q[dof_ids]
            mujoco.mj_step(model, data)

            viewer.sync()
            time_until_next_step = dt - (time.time() - step_start)
            time.sleep(0.05)


if __name__ == "__main__":
    main()
