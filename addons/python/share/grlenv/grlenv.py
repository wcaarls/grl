import ast
import numpy as np
import gym
from gym import spaces
import grlpy

class GrlEnv(gym.Env):
    def __init__(self, file):
        conf = grlpy.Configurator(file)
        inst = conf.instantiate()
        self.env = grlpy.Environment(inst["environment"])
        
        if inst["environment"]["observation_min"]:
            params = inst["environment"]
        elif inst["environment"]["task"]["observation_min"]:
            params = inst["environment"]["task"]
        else:
            raise KeyError("Can't find environment parameters")
            
        observation_min = vtoa(params["observation_min"])
        observation_max = vtoa(params["observation_max"])
        action_min = vtoa(params["action_min"])
        action_max = vtoa(params["action_max"])
        reward_min = vtoa(params["reward_min"])
        reward_max = vtoa(params["reward_max"])
        
        self.action_space = spaces.Box(low=action_min, high=action_max, dtype=np.float32)
        self.observation_space = spaces.Box(low=observation_min, high=observation_max, dtype=np.float32)
        self.reward_range = (reward_min, reward_max)

    def step(self,u):
        (obs, reward, terminal) = self.env.step(u)
        return obs, reward, terminal, {}

    def reset(self):
        return self.env.start(0)

    def render(self, mode='human'):
        pass

def vtoa(v):
    return np.array(ast.literal_eval(str(v)))
    