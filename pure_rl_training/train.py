import os
import pybullet as p
from env.spotmicro_env import SpotMicroBotEnv
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback, ProgressBarCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

# === Settings ===
total_timesteps = 10_000_000
save_freq = 100_000
eval_freq = 50_000
log_dir = "./tensorboard_logs/spotmicro_bot_ppo/"
checkpoint_dir = "./checkpoints/"
eval_dir = "./eval_logs/"
os.makedirs(log_dir, exist_ok=True)
os.makedirs(checkpoint_dir, exist_ok=True)
os.makedirs(eval_dir, exist_ok=True)

# === Define the environment factory ===
# MODIFIED: Accepts render argument, defaults to False for training
def make_env(render=False):
    def _init():
        env = SpotMicroBotEnv(render=render)
        env = Monitor(env)
        return env
    return _init

# === 1. Check Environment (Headless) ===
# Use render=False to avoid opening a window just for checking
print("Checking environment...")
check_env(make_env(render=False)(), warn=True)

# === 2. Vectorized Training Env (Headless) ===
# CRITICAL: render=False for training to prevent crash and speed up training
env = DummyVecEnv([make_env(render=False)])
env = VecNormalize(env, norm_obs=True, norm_reward=True)


# === 3. Evaluation Env (Headless) ===
# We also keep eval headless during training to avoid interrupting the process.
# You can visualize the result *after* training.
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
    name_prefix="ppo_spotmicro"
)

eval_callback = EvalCallback(
    eval_env,
    best_model_save_path=eval_dir,
    log_path=eval_dir,
    eval_freq=eval_freq,
    deterministic=True,
    render=False # Keep False here. Render in a separate script.
)

print("Starting training...")
# === Start training ===
try:
    model.learn(
        total_timesteps=total_timesteps,
        callback=[checkpoint_callback, eval_callback, ProgressBarCallback()]
    )
except KeyboardInterrupt:
    print("Training interrupted manually.")
finally:
    # === Save model + normalization statistics ===
    model.save("ppo_spotmicro_final")
    env.save("ppo_spotmicro_vecnorm.pkl")
    print("Model saved.")
    
    # Explicitly close envs to free PyBullet resources
    env.close()
    eval_env.close()