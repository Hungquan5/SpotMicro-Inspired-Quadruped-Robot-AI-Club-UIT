import os
import time
import numpy as np
import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bullet_client
import gymnasium as gym
from gymnasium import spaces

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv

# ==========================================
# 1. Define Environment
# ==========================================
class SpotMicroBalanceEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 240}

    def __init__(self, render=False, robot_urdf_path=None, init_height=0.25):
        self.render_mode = render
        self.time_step = 1.0 / 240.0
        self.max_force = 5.0
        self.num_sim_steps = 10
        self.init_height = init_height
        self.num_joints = 12 

        # Update this path if necessary
        self.robot_urdf_path = robot_urdf_path or "/home/quannh/spotmicrobot_env/simulation/main_part/urdf/spotmicroai_gen_ros.urdf"

        self.physics_cli = bullet_client.BulletClient(
            connection_mode=p.GUI if self.render_mode else p.DIRECT
        )
        self.physics_cli.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.physics_cli.setTimeStep(self.time_step)

        self.robot_id = None
        self.joint_indices = []
        self.joint_lower_limits = []
        self.joint_upper_limits = []

        # Observation Space
        obs_dim = self.num_joints * 2 + 1 + 3 + 3  
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)

        # Action Space
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(self.num_joints,), dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.physics_cli.resetSimulation()
        self.physics_cli.setGravity(0, 0, -9.81)
        self.physics_cli.loadURDF("plane.urdf")

        start_pos = [0, 0, self.init_height]
        start_ori = self.physics_cli.getQuaternionFromEuler([0.0, 0.0, 0.0])

        self.robot_id = self.physics_cli.loadURDF(
            self.robot_urdf_path, start_pos, start_ori, flags=p.URDF_USE_SELF_COLLISION
        )

        self.joint_indices = [
            i for i in range(self.physics_cli.getNumJoints(self.robot_id))
            if self.physics_cli.getJointInfo(self.robot_id, i)[2] != self.physics_cli.JOINT_FIXED
        ]

        self.joint_lower_limits = []
        self.joint_upper_limits = []
        for i in self.joint_indices:
            info = self.physics_cli.getJointInfo(self.robot_id, i)
            lower, upper = info[8], info[9]
            if lower >= upper: lower, upper = -0.5, 0.5
            self.joint_lower_limits.append(lower)
            self.joint_upper_limits.append(upper)
            self.physics_cli.resetJointState(self.robot_id, i, targetValue=0.5*(lower+upper))

        for _ in range(10):
            self.physics_cli.stepSimulation()

        return self._get_obs(), {}

    def _get_obs(self):
        joint_states = self.physics_cli.getJointStates(self.robot_id, self.joint_indices)
        joint_positions = [s[0] for s in joint_states]
        joint_velocities = [s[1] for s in joint_states]
        base_pos, base_ori = self.physics_cli.getBasePositionAndOrientation(self.robot_id)
        linear_vel, angular_vel = self.physics_cli.getBaseVelocity(self.robot_id)
        
        obs = np.concatenate([
            np.array(joint_positions, dtype=np.float32),
            np.array(joint_velocities, dtype=np.float32),
            np.array([base_pos[2]], dtype=np.float32),
            np.array(linear_vel, dtype=np.float32),
            np.array(angular_vel, dtype=np.float32)
        ])
        return obs

    def step(self, action):
        action = np.clip(action, -1.0, 1.0)
        lower = np.array(self.joint_lower_limits, dtype=np.float32)
        upper = np.array(self.joint_upper_limits, dtype=np.float32)
        valid_range = upper - lower
        center = 0.5 * (upper + lower)
        scaled_action = center + 0.5 * valid_range * action

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
        return obs, 0.0, False, False, {}

    def close(self):
        self.physics_cli.disconnect()

# ==========================================
# 2. INFERENCE LOOP
# ==========================================

def run_inference():
    # PATH FIX: Do NOT include .zip in the string
    model_path = "/home/quannh/SpotMicro-Inspired-Quadruped-Robot-AI-Club-UIT/balance_training/checkpoints_balance/ppo_spotmicro_balance_200000_steps"

    # Verify path before starting
    if not os.path.exists(model_path + ".zip"):
        print(f"Error: Model not found at {model_path}.zip")
        return

    print("Loading environment...")
    env = SpotMicroBalanceEnv(render=True)
    
    # Wrap in DummyVecEnv (Standard for SB3)
    env = DummyVecEnv([lambda: env])

    # Note: We removed VecNormalize because you do not have the .pkl file.
    
    print(f"Loading Model from {model_path}...")
    model = PPO.load(model_path, env=env)

    print("\n=== STARTING INFERENCE ===")
    print("Press Ctrl+C to stop.")

    obs = env.reset()
    
    try:
        while True:
            # Inference: Predict action based on observation
            # deterministic=True ensures the robot uses the best learned action without noise
            action, _states = model.predict(obs, deterministic=True)
            
            # Apply action to environment
            obs, rewards, dones, infos = env.step(action)
            
            if dones[0]:
                print("Robot fell/Resetting environment...")
                obs = env.reset()
                
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        env.close()

if __name__ == "__main__":
    run_inference()