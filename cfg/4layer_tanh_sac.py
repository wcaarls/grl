#!/usr/bin/python3
#
# NOTE: Actions are defined on [-1, 1], so they need to be
# normalized on input (with a signed projector/pre/normalized) and
# on output (with a renormalizing mapping/policy/action)

from __future__ import print_function

import numpy as np
import tensorflow as tf
import time, sys


if len(sys.argv) != 4:
  print("Usage:")
  print(" ", sys.argv[0], "<inputs> <outputs> <output.pb>")
  sys.exit(1)

obs = int(sys.argv[1])
actions = int(sys.argv[2])
layer1_size = 400
layer2_size = 300
log_std_min = -20
log_std_max = 2
h = -10

config = tf.ConfigProto()
config.gpu_options.per_process_gpu_memory_fraction = 0.1
session = tf.Session(config=config)

# Temperature
log_alpha = tf.get_variable(name='log_alpha', dtype=tf.float32, initializer=0.)
alpha = tf.exp(log_alpha)

# Actor network
s_in = tf.placeholder(tf.float32, shape=(None,obs), name='s_in')

prev_vars = len(tf.trainable_variables())

ha1 = tf.layers.dense(s_in, layer1_size, activation=tf.nn.relu)
ha2 = tf.layers.dense(ha1, layer2_size, activation=tf.nn.relu)
mu = tf.layers.dense(ha2, actions, activation=None)
log_std = tf.layers.dense(ha2, actions, activation=tf.tanh)
log_std = log_std_min + 0.5 * (log_std_max - log_std_min) * (log_std + 1)

a_vars = tf.trainable_variables()[prev_vars:]

pi = mu + tf.exp(log_std) * tf.random.normal(shape=tf.shape(mu))
pre_sum = -0.5 * (((pi-mu)/(tf.exp(log_std)+1e-8))**2 + 2*log_std + np.log(2*np.pi))
log_prob = tf.reduce_sum(pre_sum, axis=1, keepdims=True)

# Squashing
mu = tf.tanh(mu, name='mu')
pi = tf.tanh(pi, name='a_out')
log_prob -= tf.reduce_sum(tf.log(1 - pi**2 + 1e-6), axis=1, keepdims=True, name='log_prob')

# Critic network
a_in = tf.placeholder_with_default(pi, shape=(None,actions), name='a_in')
hc = tf.concat([s_in, a_in], axis=-1)

prev_vars = len(tf.trainable_variables())

h1q1 = tf.layers.dense(hc, layer1_size, activation=tf.nn.relu)
h2q1 = tf.layers.dense(h1q1, layer2_size, activation=tf.nn.relu)
q1 = tf.layers.dense(h2q1, 1, activation=None)
h1q2 = tf.layers.dense(hc, layer1_size, activation=tf.nn.relu)
h2q2 = tf.layers.dense(h1q2, layer2_size, activation=tf.nn.relu)
q2 = tf.layers.dense(h2q2, 1, activation=None)

q_vars = tf.trainable_variables()[prev_vars:]

q = tf.minimum(q1, q2)
v = tf.identity(q - alpha * log_prob, name='v')

tf.group([s_in, a_in], name='inputs')
tf.group([v, pi], name='outputs')

# Actor network optimization
# This should be called without feeding a_in
pi_loss = tf.reduce_mean(alpha * log_prob - q)
pi_optimize = tf.train.AdamOptimizer(3e-4).minimize(pi_loss, var_list=a_vars)

# Critic network optimization
# This should be called while feeding a_in = a_t
y_in = tf.placeholder(tf.float32, shape=(None,1), name='target')
q_loss = tf.reduce_mean((q1 - y_in)**2 + (q2 - y_in)**2)
q_optimize = tf.train.AdamOptimizer(3e-4).minimize(q_loss, var_list=q_vars, name='update')

# Temperature optimization
alpha_loss = -tf.reduce_mean(log_alpha * tf.stop_gradient(log_prob + h))
alpha_optimize = tf.train.AdamOptimizer(3e-4).minimize(alpha_loss)

a_update = tf.group([pi_optimize, alpha_optimize], name='a_update')

# Create weight assign placeholders
vars = tf.trainable_variables()
for v in vars:
  tf.assign(v, tf.placeholder(tf.float32, shape=v.shape))

# Create init node
init = tf.group([tf.global_variables_initializer()], name='init')

tf.train.write_graph(session.graph.as_graph_def(), './', sys.argv[3], as_text=False)
