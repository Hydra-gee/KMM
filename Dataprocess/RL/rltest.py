import time
import gymnasium as gym
from stable_baselines3 import PPO
import mmEnv
env = gym.make("mapmatching-v0")
model = PPO.load("ppo_mapmatching",device='cuda')
print("test")
totalDataNum = 2700
for idx in range(0, int(totalDataNum * 0.2)):
    obs,_ = env.reset(options={'trace_idx':idx})
    predictTime = 0
    while True:
        t1 = time.time()
        action, _ = model.predict(obs)
        t2 = time.time()
        predictTime += t2-t1
        obs, rewards, dones,_, info = env.step(action[0])
        if dones:
            break
    print("idx",idx,"time",predictTime)
env.save()