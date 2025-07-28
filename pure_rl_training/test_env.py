import time
import numpy as np
from env.spotmicro_env import SpotMicroBotEnv

def main():
    # Create the environment (render=True to see GUI)
    env = SpotMicroBotEnv(render=True)

    # Reset the environment
    obs = env.reset()
    print("Initial observation:", obs)

    total_reward = 0
    step_count = 0

    try:
        for _ in range(1000):  # Max steps
            # Sample a random action
            action = env.action_space.sample()

            # Step the environment
            obs, reward, done, info = env.step(action)

            total_reward += reward
            step_count += 1

            print(f"[{step_count}] Reward: {reward:.3f}, Done: {done}")

            if done:
                print("Robot fell! Resetting...")
                obs = env.reset()
                total_reward = 0
                step_count = 0

    except KeyboardInterrupt:
        print("Interrupted by user.")

    finally:
        env.close()
        print("Environment closed.")

if __name__ == "__main__":
    main()
