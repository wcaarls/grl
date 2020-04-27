#!/usr/bin/python3
#
# NOTE: Actions are defined on [-1, 1], so they need to be
# normalized on input (with a signed projector/pre/normalized) and
# on output (with a renormalizing mapping/policy/action)

from __future__ import print_function

import numpy as np
import tensorflow as tf
import time, sys

from keras.models import Model
from keras.layers.core import Dense, Lambda
from keras.layers.merge import Concatenate
from keras.layers.normalization import BatchNormalization
from keras.backend import get_session, set_session

if len(sys.argv) != 4:
  print("Usage:")
  print(" ", sys.argv[0], "<inputs> <outputs> <output.pb>")
  sys.exit(1)

obs = int(sys.argv[1])
actions = int(sys.argv[2])
normalization = False
share_weights = False
layer1_size = 400
layer2_size = 300

config = tf.ConfigProto()
config.gpu_options.per_process_gpu_memory_fraction = 0.1
session = tf.Session(config=config)
set_session(session)

# Actor network definition
s_in = tf.placeholder(tf.float32, shape=(None,obs), name='s_in')
if normalization:
  sn = BatchNormalization()(s_in)
else:
  sn = s_in
hc = Dense(layer1_size, activation='relu', name='h_common')(sn)
if normalization:
  hcn = BatchNormalization()(hc)
else:
  hcn = hc
ha = Dense(layer2_size, activation='relu', name='h_actor')(hcn)
if normalization:
  han = BatchNormalization()(ha)
else:
  han = ha
a_out = tf.identity(Dense(actions, activation='tanh')(han), name='a_out')
theta = tf.trainable_variables()

# DDPG->TD3: add target noise
a_out_noise = a_out + tf.clip_by_value(tf.random.normal(shape=tf.shape(a_out))*0.1, -0.2, 0.2)
a_in = tf.placeholder_with_default(tf.stop_gradient(a_out_noise), shape=(None,actions), name='a_in')

# First critic network definition
if normalization:
  an = BatchNormalization()(a_in)
else:
  an = a_in
if share_weights:  
  ca = Concatenate()([hcn, an])
else:
  hc2 = Dense(layer1_size, activation='relu', name='h_common2_1')(sn)
  if normalization:
    hcn2 = BatchNormalization()(hc2)
  else:
    hcn2 = hc2
  ca = Concatenate()([hcn2, an])
hq = Dense(layer2_size, activation='relu', name='h_critic_1')(ca)
if normalization:
  hqn = BatchNormalization()(hq)
else:
  hqn = hq
q1 = Dense(1, activation='linear', name='q_1')(hqn)

# Second network definition
if normalization:
  an = BatchNormalization()(a_in)
else:
  an = a_in
if share_weights:  
  ca = Concatenate()([hcn, an])
else:
  hc2 = Dense(layer1_size, activation='relu', name='h_common2_2')(sn)
  if normalization:
    hcn2 = BatchNormalization()(hc2)
  else:
    hcn2 = hc2
  ca = Concatenate()([hcn2, an])
hq = Dense(layer2_size, activation='relu', name='h_critic_2')(ca)
if normalization:
  hqn = BatchNormalization()(hq)
else:
  hqn = hq
q2 = Dense(1, activation='linear', name='q_2')(hqn)

# DDPG->TD3: uses minimum of two Q functions
q = tf.identity(tf.minimum(q1, q2), name='q' )

tf.group([s_in, a_in], name='inputs')
tf.group([q, a_out], name='outputs')

# Critic network update
q_target = tf.placeholder(tf.float32, shape=(None, 1), name='target')
q_loss = tf.losses.mean_squared_error(q_target, q1) + tf.losses.mean_squared_error(q_target, q2)
q_update = tf.train.AdamOptimizer(0.001).minimize(q_loss, name='update')

# Actor network update
dq_da = tf.gradients(q1, a_in, name='dq_da')[0]
dq_dtheta = tf.gradients(a_out, theta, -dq_da, name='dq_dtheta')

a_update = tf.train.AdamOptimizer(0.0001).apply_gradients(zip(dq_dtheta, theta), name='a_update')

# Create weight assign placeholders
vars = tf.trainable_variables()
for v in vars:
  tf.assign(v, tf.placeholder(tf.float32, shape=v.shape))

tf.train.write_graph(get_session().graph.as_graph_def(), './', sys.argv[3], as_text=False)
