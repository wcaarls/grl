#!/usr/bin/python3
#
# OpenAI Gym example using GRL agent
# This example must be run from its own directory, with
# grl installed to a path in LD_LIBRARY_PATH and grlpy installed
# to a path in PYTHON_PATH.

import gym
import grlpy

# Create GRL agent
conf = grlpy.Configurator("../cfg/sarsa_sincos.yaml")
inst = conf.instantiate()
agent = grlpy.Agent(inst["agent"])

# Create Gym environment
env = gym.make("Pendulum-v0")

# Run episodes
for ep in range(1000):
  observation = env.reset()
  action = agent.start(observation)
  done = False
  while not done:
    if not ep%100:
      env.render()
    observation, reward, done, info = env.step(action)
    
    if done:
      action = agent.end(observation, reward)
    else:
      action = agent.step(observation, reward)
