import sys
import time

# Assumes this script is being run from grl/bin
sys.path.append('../build')

import grlpy

# Load configuration
conf = grlpy.Configurator("../cfg/pendulum/sarsa_tc.yaml")

# Instantiate configuration (construct objects)
inst = conf.instantiate()

# Get reference to agent and environment
agent = grlpy.Agent(inst["experiment"]["agent"])
env = grlpy.Environment(inst["experiment"]["environment"])

# 2000 episodes
for r in range(2000):
  terminal = 0
  
  # Restart environment and agent
  obs = env.start(0)
  action = agent.start(obs)
  
  # Run episode
  while not terminal:
    (obs, reward, terminal) = env.step(action)
    if terminal:
      agent.end(obs, reward)
    else:
      action = agent.step(obs, reward)
