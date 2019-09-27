#!/usr/bin/python3
#
# NOTE: Actions are defined on [-1, 1], so they need to be
# normalized on input (with a signed projector/pre/normalized) and
# on output (with a renormalizing mapping/policy/action)

from __future__ import print_function

import numpy as np
import tensorflow as tf
import time, sys, random

from keras.models import Model
from keras.layers.core import Dense, Lambda
from keras.layers.merge import Concatenate
from keras.layers.normalization import BatchNormalization
from keras.backend import get_session

if len(sys.argv) != 5:
  print("Usage:")
  print(" ", sys.argv[0], "<inputs> <outputs> <networks> <output.pb>")
  sys.exit(1)

obs = int(sys.argv[1])
actions = int(sys.argv[2])
networks = int(sys.argv[3])
normalization = False
share_weights = False
layer1_minsize = 400
layer1_maxsize = 800
layer2_minsize = 300
layer2_maxsize = 600
lr_critic = 0.001
lr_actor = 0.0001

# Input
s_in = tf.placeholder(tf.float32, shape=(None,obs), name='s_in')
a_in = tf.placeholder_with_default(tf.zeros(shape=(0,actions), dtype=tf.float32), shape=(None,actions), name='a_in')
tf.group([s_in, a_in], name='inputs')
a_in_set = tf.not_equal(tf.size(a_in), 0)
q_target = tf.placeholder(tf.float32, shape=(None, networks), name='target')

q_gradients = []
a_gradients = []
q_all = []
a_all = []

q_opt = tf.train.AdamOptimizer(lr_critic)
a_opt = tf.train.AdamOptimizer(lr_actor)

for n in range(networks):
  if n == 0:
    layer1_size = layer1_minsize
    layer2_size = layer2_minsize
  else:
    layer1_size = layer1_maxsize
    layer2_size = layer2_maxsize
  
  print (layer1_size, layer2_size)

  # Actor network definition
  if normalization:
    sn = BatchNormalization()(s_in, name='net%d_s_norm' % n)
  else:
    sn = s_in
  hc = Dense(layer1_size, activation='relu', name='net%d_h_common' % n)(sn)
  if normalization:
    hcn = BatchNormalization()(hc, name='net%d_h_common_norm')
  else:
    hcn = hc
  ha = Dense(layer2_size, activation='relu', name='net%d_h_actor' % n)(hcn)
  if normalization:
    han = BatchNormalization()(ha, name='net%d_h_actor_norm' % n)
  else:
    han = ha
  a_out = Dense(actions, activation='tanh', name='net%d_a_out' % n)(han)
  a_all.append(a_out)
  
  theta = tf.trainable_variables('net%d_' % n)

  # Critic network definition
  a_cond = tf.cond(a_in_set, lambda: a_in, lambda: a_out, name='net%d_a_cond' % n)
  if normalization:
    an = BatchNormalization()(a_cond, name='net%d_a_norm' % n)
  else:
    an = a_cond
  if share_weights:  
    ca = Concatenate()([hcn, an])
  else:
    hc2 = Dense(layer1_size, activation='relu', name='net%d_h_common2' % n)(sn)
    if normalization:
      hcn2 = BatchNormalization()(hc2, name='net%d_h_common2_norm' % n)
    else:
      hcn2 = hc2
    ca = Concatenate()([hcn2, an])
  hq = Dense(layer2_size, activation='relu', name='net%d_h_critic' % n)(ca)
  if normalization:
    hqn = BatchNormalization()(hq, name='net%d_h_critic_norm' % n)
  else:
    hqn = hq
  q = Dense(1, activation='linear', name='net%d_q' % n)(hqn)
  q_all.append(q)
  
  # Critic network gradients
  q_loss = tf.losses.mean_squared_error(tf.expand_dims(q_target[:,n], 1), q)
  q_gradients.extend(q_opt.compute_gradients(q_loss))

  # Actor network gradients
  dq_da = tf.gradients(q, a_cond, name='net%d_dq_da' % n)[0]
  dq_dtheta = tf.gradients(a_out, theta, -dq_da, name='net%d_dq_dtheta' % n)
  a_gradients.extend(zip(dq_dtheta, theta))
  
# Aggregate outputs
q = tf.concat(q_all, 1, name='q')
a_out = tf.concat(a_all, 1, name='a_out')
tf.group([q, a_out], name='outputs')

# Updates
q_update = q_opt.apply_gradients(q_gradients, name='update')
a_update = a_opt.apply_gradients(a_gradients, name='a_update')

# Create weight assign placeholders
vars = tf.trainable_variables()
for v in vars:
  tf.assign(v, tf.placeholder(tf.float32, shape=v.shape))

tf.train.write_graph(get_session().graph.as_graph_def(), './', sys.argv[4], as_text=False)
