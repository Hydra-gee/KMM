import sys
import gymnasium as gym
from stable_baselines3 import PPO, DQN
import mmEnv

env = gym.make("mapmatching-v0")
model = PPO("MlpPolicy", env, verbose=1,device='cuda',tensorboard_log="./tensorboard_logs/")
# model = PPO.load("ppo_mapmatching",env=env)
model.learn(total_timesteps=3000,tb_log_name="shanghai")
model.save("ppo_mapmatching")
env.save()
print("end")
sys.exit()
