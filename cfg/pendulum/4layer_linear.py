#!/usr/bin/python

from __future__ import print_function

import numpy as np
import tensorflow as tf
import sys

from keras.layers.core import Dense
from keras.backend import get_session

if len(sys.argv) != 4:
  print("Usage:")
  print(" ", sys.argv[0], "<inputs> <outputs> <output.pb>")
  sys.exit(1)

inputs = int(sys.argv[1])
outputs = int(sys.argv[2])
layer1_size = 400
layer2_size = 300

inp = tf.placeholder(tf.float32, shape=(None,inputs), name='input')
h1 = Dense(layer1_size, activation='relu', name='h1')(inp)
h2 = Dense(layer2_size, activation='relu', name='h2')(h1)
outp = Dense(outputs, activation='linear', name='output')(h2)

tf.group(inp, name='inputs')
tf.group(outp, name='outputs')

target = tf.placeholder(tf.float32, shape=(None, outputs), name='target')
loss = tf.losses.mean_squared_error(target, outp)
update = tf.train.AdamOptimizer(0.001).minimize(loss, name='update')

# Create weight assign placeholders
vars = tf.trainable_variables()
for v in vars:
  tf.assign(v, tf.placeholder(tf.float32, shape=v.shape))

tf.train.write_graph(get_session().graph.as_graph_def(), './', sys.argv[3], as_text=False)
