import numpy as np
import genesis as gs

########################## init ##########################
gs.init(backend=gs.cpu)

########################## create a scene ##########################
scene = gs.Scene(
    viewer_options = gs.options.ViewerOptions(
        camera_pos    = (0, -3.5, 2.5),
        camera_lookat = (0.0, 0.0, 0.5),
        camera_fov    = 30,
        max_FPS       = 60,
    ),
    sim_options = gs.options.SimOptions(
        dt = 0.01,
    ),
    show_viewer = True,
)

########################## entities ##########################
plane = scene.add_entity(gs.morphs.Plane())

robot = scene.add_entity(
    gs.morphs.URDF(
        file  = '/home/quannh/spotmicrobot_env/simulation/main_part/urdf/spotmicroai_gen_ros.urdf',
        pos   = (1.0, 1.0, 0.0),
        euler = (0, 0, 0),
    )
)

scene.build()

# === Joint names ===
jnt_names = [
    'joint_base_link',
    'front_left_shoulder',
    'front_right_shoulder',
    'rear_left_shoulder',
    'rear_right_shoulder',
    'front_left_leg',
    'front_right_leg',
    'rear_left_leg',
    'rear_right_leg',
    'front_left_foot',
    'front_right_foot',
    'rear_left_foot',
    'rear_right_foot',
]

# === Get DoF indices ===
dofs_idx_raw = [robot.get_joint(name).dof_idx_local for name in jnt_names]
flat_dofs_idx = list(range(6, 18))  # skip base (6-DoF), use 12 revolute joints

# === Hard reset motion ===
for i in range(150):
    if i < 50:
        robot.set_dofs_position(np.array([1, 1, 0, 0, 0, 0, 0, 0.04, 0.04, 0, 0, 0]), flat_dofs_idx)
    elif i < 100:
        robot.set_dofs_position(np.array([-1, 0.8, 1, -2, 1, 0.5, -0.5, 0.04, 0.04, 0, 0, 0]), flat_dofs_idx)
    else:
        robot.set_dofs_position(np.zeros(12), flat_dofs_idx)
    scene.step()

# === PD control phase ===
for i in range(1250):
    if i == 0:
        robot.control_dofs_position(np.array([1, 1, 0, 0, 0, 0, 0, 0.04, 0.04, 0, 0, 0]), flat_dofs_idx)
    elif i == 250:
        robot.control_dofs_position(np.array([-1, 0.8, 1, -2, 1, 0.5, -0.5, 0.04, 0.04, 0, 0, 0]), flat_dofs_idx)
    elif i == 500:
        robot.control_dofs_position(np.zeros(12), flat_dofs_idx)
    elif i == 750:
        # Velocity for first joint, position for rest
        robot.control_dofs_position(np.zeros(11), flat_dofs_idx[1:])
        robot.control_dofs_velocity(np.array([1.0]), [flat_dofs_idx[0]])
    elif i == 1000:
        robot.control_dofs_force(np.zeros(12), flat_dofs_idx)

    print('control force:', robot.get_dofs_control_force(flat_dofs_idx))
    print('internal force:', robot.get_dofs_force(flat_dofs_idx))

    scene.step()
