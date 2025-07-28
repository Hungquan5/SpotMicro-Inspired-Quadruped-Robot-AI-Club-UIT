import time
import numpy as np
from env.spotmicro_env import SpotMicroBotEnv  # make sure this path matches your actual module file

# Initialize environment
env = SpotMicroBotEnv(render=True)

# Reset environment
obs, _ = env.reset()

# Parameters
num_steps_per_joint = 100
sleep_time = 1. / 60.

# Move each joint one by one
for joint_id in range(len(env.joint_indices)):
    print(f"\n[INFO] Moving joint {joint_id}: {env.joint_indices[joint_id]}")
    
    for step in range(num_steps_per_joint):
        # Generate sinusoidal position for the selected joint
        angle = np.sin(2 * np.pi * step / num_steps_per_joint)

        # Create action array with zeros
        action = np.zeros(len(env.joint_indices), dtype=np.float32)
        action[joint_id] = angle  # Only move one joint at a time

        # Apply the action
        obs, reward, done, truncated, info = env.step(action)
        
        time.sleep(sleep_time)

print("\n[INFO] Joint debug complete.")
env.physics_cli.disconnect()
