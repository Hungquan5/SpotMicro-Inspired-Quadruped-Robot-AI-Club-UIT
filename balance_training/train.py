# spotmicro_balance_train.py
import os
import time
import math
import numpy as np
import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bullet_client

import gymnasium as gym
from gymnasium import spaces

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback, ProgressBarCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

# === Settings ===
total_timesteps = 4_000_000     # balance pretraining can be shorter; increase if you want
save_freq = 200_000
eval_freq = 100_000
log_dir = "./tensorboard_logs/spotmicro_balance_ppo/"
checkpoint_dir = "./checkpoints_balance/"
eval_dir = "./eval_logs_balance/"
os.makedirs(log_dir, exist_ok=True)
os.makedirs(checkpoint_dir, exist_ok=True)
os.makedirs(eval_dir, exist_ok=True)

# === Environment ===
class SpotMicroBalanceEnv(gym.Env):
    """
    Gymnasium environment for balance-only training of SpotMicro.
    """
    metadata = {"render_modes": ["human"], "render_fps": 240}

    def __init__(self, render=False, robot_urdf_path=None, init_height=0.25):
        self.render_mode = render
        self.time_step = 1.0 / 240.0
        self.max_force = 5.0
        self.num_sim_steps = 10
        self.init_height = init_height
        
        # Standard SpotMicro has 12 joints
        self.num_joints = 12 

        # Default URDF path
        self.robot_urdf_path = robot_urdf_path or "/home/quannh/spotmicrobot_env/simulation/main_part/urdf/spotmicroai_gen_ros.urdf"

        self.physics_cli = bullet_client.BulletClient(
            connection_mode=p.GUI if self.render_mode else p.DIRECT
        )
        self.physics_cli.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.physics_cli.setTimeStep(self.time_step)

        # Will be set during reset
        self.robot_id = None
        self.joint_indices = []
        self.joint_lower_limits = []
        self.joint_upper_limits = []

        # --- FIX STARTS HERE ---
        # Define Observation Space immediately in __init__
        # 12 pos + 12 vel + 1 height + 3 lin_vel + 3 ang_vel = 31
        obs_dim = self.num_joints * 2 + 1 + 3 + 3  
        self.observation_space = spaces.Box(
            low=-np.inf, 
            high=np.inf, 
            shape=(obs_dim,), 
            dtype=np.float32
        )

        # Define Action Space immediately in __init__
        self.action_space = spaces.Box(
            low=-1.0, 
            high=1.0, 
            shape=(self.num_joints,), 
            dtype=np.float32
        )
        # --- FIX ENDS HERE ---

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # Reset simulation
        self.physics_cli.resetSimulation()
        self.physics_cli.setGravity(0, 0, -9.81)
        self.physics_cli.loadURDF("plane.urdf")

        start_pos = [0, 0, self.init_height]
        start_ori = self.physics_cli.getQuaternionFromEuler([0.0, 0.0, 0.0])

        flags = p.URDF_USE_SELF_COLLISION
        self.robot_id = self.physics_cli.loadURDF(
            self.robot_urdf_path,
            start_pos,
            start_ori,
            flags=flags
        )

        # collect non-fixed joints
        self.joint_indices = [
            i for i in range(self.physics_cli.getNumJoints(self.robot_id))
            if self.physics_cli.getJointInfo(self.robot_id, i)[2] != self.physics_cli.JOINT_FIXED
        ]

        # Safety check: ensure loaded URDF matches our assumption
        if len(self.joint_indices) != self.num_joints:
            print(f"WARNING: Loaded robot has {len(self.joint_indices)} joints, expected {self.num_joints}.")

        # joint limits (safe fallback if limits are not set)
        self.joint_lower_limits = []
        self.joint_upper_limits = []
        for i in self.joint_indices:
            info = self.physics_cli.getJointInfo(self.robot_id, i)
            lower = info[8]
            upper = info[9]
            if lower >= upper:
                lower, upper = -0.5, 0.5
            self.joint_lower_limits.append(lower)
            self.joint_upper_limits.append(upper)
            center = 0.5 * (lower + upper)
            self.physics_cli.resetJointState(self.robot_id, i, targetValue=center)

        for _ in range(10):
            self.physics_cli.stepSimulation()

        # --- REMOVED SPACE RE-DEFINITION HERE ---
        # Use the spaces defined in __init__

        return self._get_obs(), {}

    def _get_obs(self):
        # joint states
        joint_states = self.physics_cli.getJointStates(self.robot_id, self.joint_indices)
        joint_positions = [s[0] for s in joint_states]
        joint_velocities = [s[1] for s in joint_states]

        base_pos, base_ori = self.physics_cli.getBasePositionAndOrientation(self.robot_id)
        linear_vel, angular_vel = self.physics_cli.getBaseVelocity(self.robot_id)

        # use base height as direct signal (important for balance)
        base_height = base_pos[2]
        # use roll/pitch/yaw if you prefer; here we use angular_vel which is crucial for balance
        obs = np.concatenate([
            np.array(joint_positions, dtype=np.float32),
            np.array(joint_velocities, dtype=np.float32),
            np.array([base_height], dtype=np.float32),
            np.array(linear_vel, dtype=np.float32),
            np.array(angular_vel, dtype=np.float32)
        ])
        return obs

    def step(self, action):
        action = np.clip(action, -1.0, 1.0)
        # scale action to joint limits, with safe fallback if limits invalid
        lower = np.array(self.joint_lower_limits, dtype=np.float32)
        upper = np.array(self.joint_upper_limits, dtype=np.float32)

        # If any upper==lower, use small delta around current joint position as fallback
        valid_range = upper - lower
        scaled_action = np.zeros_like(action, dtype=np.float32)
        # center positions
        center = 0.5 * (upper + lower)
        # use action to move within ±50% of the allowed range around center
        scaled_action = center + 0.5 * valid_range * action

        # If any invalid (zero range), fallback to small increments around current pos
        for idx, vr in enumerate(valid_range):
            if abs(vr) < 1e-6:
                js = self.physics_cli.getJointState(self.robot_id, self.joint_indices[idx])
                cur = js[0]
                scaled_action[idx] = cur + 0.05 * float(action[idx])

        # apply position control
        for i, joint_index in enumerate(self.joint_indices):
            target = float(scaled_action[i])
            self.physics_cli.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target,
                force=self.max_force
            )

        # step sim forward
        for _ in range(self.num_sim_steps):
            self.physics_cli.stepSimulation()
            if self.render_mode:
                time.sleep(self.time_step)

        obs = self._get_obs()
        reward = self._compute_reward()
        done = self._check_termination()
        truncated = False
        info = {}

        return obs, float(reward), bool(done), bool(truncated), info

    def _compute_reward(self):
        # base pose/vel
        base_pos, base_ori = self.physics_cli.getBasePositionAndOrientation(self.robot_id)
        linear_vel, angular_vel = self.physics_cli.getBaseVelocity(self.robot_id)
        roll, pitch, yaw = self.physics_cli.getEulerFromQuaternion(base_ori)

        # balance signals
        # keep base height near init_height
        height_error = abs(base_pos[2] - self.init_height)
        height_reward = max(0.0, 1.0 - 5.0 * height_error)  # in [ -inf, 1 ] but clipped later

        # small roll/pitch is good
        attitude_penalty = (abs(roll) + abs(pitch))  # radians
        attitude_reward = -2.0 * attitude_penalty  # stronger penalty for tilt

        # penalize angular velocity (fast rotation is unstable)
        ang_vel = np.linalg.norm(angular_vel)
        ang_vel_penalty = -1.0 * ang_vel

        # penalize excessive torque (energy usage)
        joint_states = self.physics_cli.getJointStates(self.robot_id, self.joint_indices)
        torques = np.array([abs(s[3]) for s in joint_states], dtype=np.float32)
        torque_penalty = -0.01 * np.sum(torques)

        # small survival bonus to encourage standing still long enough
        survival = 0.05

        reward = height_reward + attitude_reward + ang_vel_penalty + torque_penalty + survival

        # clip reward into a reasonable range to stabilize learning
        reward = float(np.clip(reward, -10.0, 10.0))
        return reward

    def _check_termination(self):
        pos, ori = self.physics_cli.getBasePositionAndOrientation(self.robot_id)
        roll, pitch, yaw = self.physics_cli.getEulerFromQuaternion(ori)

        # fall if base too low
        if pos[2] < 0.12:
            return True
        # fall if robot flipping too far
        if abs(roll) > 0.9 or abs(pitch) > 0.9:
            return True
        return False

    def render(self):
        # left blank; PyBullet GUI is managed by BulletClient with render_mode=True
        pass

    def close(self):
        try:
            self.physics_cli.disconnect()
        except Exception:
            pass

# === Environment factory ===
def make_env(render=False):
    def _init():
        env = SpotMicroBalanceEnv(render=render)
        env = Monitor(env)
        return env
    return _init

# === Quick env check (headless) ===
print("Checking environment...")
check_env(make_env(render=False)(), warn=True)

# === Vectorized training env (headless) ===
env = DummyVecEnv([make_env(render=False)])
env = VecNormalize(env, norm_obs=True, norm_reward=True)

# === Evaluation env (headless) ===
eval_env = DummyVecEnv([make_env(render=False)])
eval_env = VecNormalize(eval_env, norm_obs=True, norm_reward=False, training=False)

# === Model ===
model = PPO(
    policy="MlpPolicy",
    env=env,
    verbose=1,
    tensorboard_log=log_dir,
    n_steps=2048,
    batch_size=64,
    gae_lambda=0.95,
    gamma=0.99,
    n_epochs=10,
    ent_coef=0.0,
    learning_rate=2.5e-4,
    clip_range=0.2
)

# === Callbacks ===
checkpoint_callback = CheckpointCallback(
    save_freq=save_freq,
    save_path=checkpoint_dir,
    name_prefix="ppo_spotmicro_balance"
)

eval_callback = EvalCallback(
    eval_env,
    best_model_save_path=eval_dir,
    log_path=eval_dir,
    eval_freq=eval_freq,
    deterministic=True,
    render=False
)

print("Starting balance-only training...")
try:
    model.learn(
        total_timesteps=total_timesteps,
        callback=[checkpoint_callback, eval_callback, ProgressBarCallback()]
    )
except KeyboardInterrupt:
    print("Training interrupted manually.")
finally:
    model.save("ppo_spotmicro_balance_final")
    env.save("ppo_spotmicro_balance_vecnorm.pkl")
    print("Model and normalization saved.")
    env.close()
    eval_env.close()
