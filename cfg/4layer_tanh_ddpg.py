#!/usr/bin/python3
#
# NOTE: Actions are defined on [-1, 1], so they need to be
# normalized on input (with a signed projector/pre/normalized) and
# on output (with a renormalizing mapping/policy/action)

from __future__ import print_function

import time, sys
import numpy as np
import tensorflow as tf2
import tensorflow.compat.v1 as tf

from tensorflow.keras.layers import Dense, Lambda
from tensorflow.keras.layers import Concatenate
from tensorflow.keras.layers import BatchNormalization

tf2.config.set_visible_devices([], 'GPU')
tf.disable_eager_execution()

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
a_out = Dense(actions, activation='tanh', name='a_out')(han)
theta = tf.trainable_variables()

# Critic network definition
a_in = tf.placeholder_with_default(tf.stop_gradient(a_out), shape=(None,actions), name='a_in')
if normalization:
  an = BatchNormalization()(a_in)
else:
  an = a_in
if share_weights:  
  ca = Concatenate()([hcn, an])
else:
  hc2 = Dense(layer1_size, activation='relu', name='h_common2')(sn)
  if normalization:
    hcn2 = BatchNormalization()(hc2)
  else:
    hcn2 = hc2
  ca = Concatenate()([hcn2, an])
hq = Dense(layer2_size, activation='relu', name='h_critic')(ca)
if normalization:
  hqn = BatchNormalization()(hq)
else:
  hqn = hq
q = Dense(1, activation='linear', name='q')(hqn)

tf.group([s_in, a_in], name='inputs')
tf.group([q, a_out], name='outputs')

# Critic network update
q_target = tf.placeholder(tf.float32, shape=(None, 1), name='target')
q_loss = tf.losses.mean_squared_error(q_target, q)
q_update = tf.train.AdamOptimizer(0.001).minimize(q_loss, name='update')

# Actor network update
dq_da = tf.gradients(q, a_in, name='dq_da')[0]
dq_dtheta = tf.gradients(a_out, theta, -dq_da, name='dq_dtheta')

a_update = tf.train.AdamOptimizer(0.0001).apply_gradients(zip(dq_dtheta, theta), name='a_update')

# Create global variable initializer
tf.global_variables_initializer()

# Create weight read and assign placeholders
vars = tf.trainable_variables()
for v in vars:
  v.read_value()
  tf.assign(v, tf.placeholder(tf.float32, shape=v.shape))

tf.train.write_graph(tf.get_default_graph().as_graph_def(), './', sys.argv[3], as_text=False)
