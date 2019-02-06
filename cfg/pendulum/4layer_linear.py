#!/usr/bin/python2.7

from __future__ import print_function

import numpy as np
import tensorflow as tf
import sys

from keras.models import Sequential
from keras.layers.core import Dense, Activation
from keras.optimizers import SGD, Adam
from keras.backend import get_session

if len(sys.argv) != 4:
  print("Usage:")
  print(" ", sys.argv[0], "<inputs> <outputs> <output.pb>")
  sys.exit(1)

model = Sequential()
model.add(Dense(100, input_shape=(int(sys.argv[1]),)))
model.add(Activation('relu'))
model.add(Dense(100))
model.add(Activation('relu'))
model.add(Dense(int(sys.argv[2])))
model.add(Activation('linear'))

model.compile(loss='mean_squared_error', optimizer=Adam())

# Must be in this order to enable automatic discovery
model.model._make_train_function()
model.model._make_predict_function()

# Make sure weight assign placeholders are created
weights = model.get_weights()
model.set_weights(weights)

tf.train.write_graph(get_session().graph.as_graph_def(), './', sys.argv[3], as_text=False)
