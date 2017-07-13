import numpy as np
import tensorflow as tf

from keras.models import Sequential
from keras.layers.core import Dense, Activation
from keras.optimizers import SGD
from keras.backend import get_session

model = Sequential()
model.add(Dense(100, input_shape=(3,)))
model.add(Activation('tanh'))
model.add(Dense(1))
model.add(Activation('linear'))

model.summary()

model.compile(loss='mean_squared_error', optimizer=SGD())

# Must be in this order to enable automatic discovery
model.model._make_train_function()
model.model._make_predict_function()

# Make sure weight assign placeholders are created
weights = model.get_weights()
model.set_weights(weights)

tf.train.write_graph(get_session().graph.as_graph_def(), './', 'pendulum_q.pb', as_text=False)
