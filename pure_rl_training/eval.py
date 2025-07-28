from env.spotmicro_env import SpotMicroBotEnv
from stable_baselines3 import PPO 

model = PPO.load("/home/quannh/spotmicrobot_env/pure_rl_training/checkpoints/ppo_spotmicro_1000000_steps.zip")

env = SpotMicroBotEnv(render=True)
obs,_ =env.reset()

while True:
    action, _ = model.predict(obs)
    obs, reward, done, _,_ = env.step(action=action)
    if done: 
        obs,_= env.reset()