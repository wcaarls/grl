#!/usr/bin/python3
#
# GRL experiment example

import gym
import grlpy

# Create GRL agent
conf = grlpy.Configurator("../cfg/pendulum_sarsa.yaml")
inst = conf.instantiate()
exp = grlpy.Experiment(inst["experiment"])
exp.run()
exp.reset()
exp.run()
