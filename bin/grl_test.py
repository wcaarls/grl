import sys
import time

# Assumes this script is being run from grl/bin
sys.path.append('../build')

import grlpy

# Load configurations
envconf = grlpy.Configurator("../cfg/matlab/pendulum_swingup.yaml")
agentconf = grlpy.Configurator("../cfg/matlab/sarsa.yaml")

# Instantiate configurations (construct objects)
envinst = envconf.instantiate()
agentinst = agentconf.instantiate()

# Get reference to environment and agent
env = grlpy.Environment(envinst["environment"])
agent = grlpy.Agent(agentinst["agent"])

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
