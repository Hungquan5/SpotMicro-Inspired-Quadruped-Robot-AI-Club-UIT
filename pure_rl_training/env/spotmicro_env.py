import math
import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bullet_client

class SpotMicroBotEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 240}

    def __init__(self, render=False, robot_urdf_path=None):
        self.render_mode = render
        self.time_step = 1.0 / 240.0
        self.max_force = 5.0
        self.num_sim_steps = 10
        self.init_height = 0.25

        self.robot_urdf_path = robot_urdf_path or "/home/quannh/spotmicrobot_env/simulation/main_part/urdf/spotmicroai_gen_ros.urdf"

        self.physics_cli = bullet_client.BulletClient(
            connection_mode=p.GUI if self.render_mode else p.DIRECT
        )
        self.physics_cli.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.physics_cli.setTimeStep(self.time_step)

        # Will be filled on reset
        self.robot_id = None
        self.joint_indices = []
        self.joint_lower_limits = []
        self.joint_upper_limits = []

        # Observation space: joint pos (12) + vel (12) + base pos (3) + base ori (3) + base lin vel (3) = 33
        obs_dim = 12 + 12 + 3 + 3 + 3
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(12,), dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self.physics_cli.resetSimulation()
        self.physics_cli.setGravity(0, 0, -9.81)
        self.physics_cli.loadURDF("plane.urdf")

        start_pos = [0, 0, self.init_height]
        start_ori = self.physics_cli.getQuaternionFromEuler([0.0, 0.0, 0.0])

        self.robot_id = self.physics_cli.loadURDF(
            self.robot_urdf_path,
            start_pos,
            start_ori,
            flags=p.URDF_USE_SELF_COLLISION
        )

        self.joint_indices = [
            i for i in range(self.physics_cli.getNumJoints(self.robot_id))
            if self.physics_cli.getJointInfo(self.robot_id, i)[2] != self.physics_cli.JOINT_FIXED
        ]

        self.joint_lower_limits = []
        self.joint_upper_limits = []
        for i in self.joint_indices:
            joint_info = self.physics_cli.getJointInfo(self.robot_id, i)
            self.joint_lower_limits.append(joint_info[8])
            self.joint_upper_limits.append(joint_info[9])
            self.physics_cli.resetJointState(self.robot_id, i, targetValue=0.0)

        # Small stabilization steps
        for _ in range(5):
            self.physics_cli.stepSimulation()

        return self._get_obs(), {}

    def _get_obs(self):
        joint_states = self.physics_cli.getJointStates(self.robot_id, self.joint_indices)
        joint_positions = [s[0] for s in joint_states]
        joint_velocities = [s[1] for s in joint_states]

        base_pos, base_ori = self.physics_cli.getBasePositionAndOrientation(self.robot_id)
        linear_vel, _ = self.physics_cli.getBaseVelocity(self.robot_id)
        euler_ori = self.physics_cli.getEulerFromQuaternion(base_ori)

        obs = np.array(
            joint_positions + joint_velocities + list(base_pos) + list(euler_ori) + list(linear_vel),
            dtype=np.float32
        )
        return obs

    def step(self, action):
        action = np.clip(action, -1.0, 1.0)
        scaled_action = 0.5 * (np.array(self.joint_upper_limits) - np.array(self.joint_lower_limits)) * action + \
                        0.5 * (np.array(self.joint_upper_limits) + np.array(self.joint_lower_limits))

        for i, joint_index in enumerate(self.joint_indices):
            self.physics_cli.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=scaled_action[i],
                force=self.max_force
            )

        for _ in range(self.num_sim_steps):
            self.physics_cli.stepSimulation()
            if self.render_mode:
                time.sleep(self.time_step)

        obs = self._get_obs()
        reward = self._compute_reward()
        done = self._check_termination()
        truncated = False
        info = {}

        return obs, reward, done, truncated, info

    def _compute_reward(self):
        linear_vel, angular_vel = self.physics_cli.getBaseVelocity(self.robot_id)
        base_pos, base_ori = self.physics_cli.getBasePositionAndOrientation(self.robot_id)
        roll, pitch, yaw = self.physics_cli.getEulerFromQuaternion(base_ori)

        forward_vel = linear_vel[0]  # assuming +X is forward
        upright_bonus = np.clip((base_pos[2] - 0.1) * 5.0, -1.0, 1.0)
        stability_penalty = - (abs(roll) + abs(pitch))

        # joint_states = self.physics_cli.getJointStates(self.robot_id, self.joint_indices)
        # torques = [abs(state[3]) for state in joint_states]
        # energy_penalty = -0.005 * np.sum(torques)

        # ang_vel_penalty = -0.1 * np.linalg.norm(angular_vel)
        # yaw_penalty = -0.05 * abs(yaw) if abs(yaw) > 0.5 else 0.0

        reward = (
            -1.0 * forward_vel +
            0.5 * upright_bonus +
            0.3 * stability_penalty +
            0.05  # survival bonus
        )

        return reward



    def _check_termination(self):
        pos, ori = self.physics_cli.getBasePositionAndOrientation(self.robot_id)
        roll, pitch, yaw = self.physics_cli.getEulerFromQuaternion(ori)

        if pos[2] < 0.08:  # lower threshold slightly
            return True
        if abs(roll) > 1.2 or abs(pitch) > 1.2:  # ~68 degrees instead of 57
            return True
        return False


    def render(self):
        pass  # Not needed, GUI handled by PyBullet

    def close(self):
        self.physics_cli.disconnect()
