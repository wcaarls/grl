import numpy as np
import tensorflow as tf
import time

from keras import backend as K
from keras.models import Model
from keras.layers import Input
from keras.layers.core import Dense, Lambda
from keras.layers.merge import Multiply, Add
from keras.layers.normalization import BatchNormalization
from keras.optimizers import SGD, Adam, RMSprop
from keras.backend import get_session

inp = Input(shape=(3,))
s = Lambda(lambda x: x[:,0:2], output_shape=(2,), name='s')(inp)
a = Lambda(lambda x: x[:,2:3], output_shape=(1,), name='a')(inp)
s = BatchNormalization()(s)
h1 = Dense(100, activation='relu', name='h1')(s)
h1 = BatchNormalization()(h1)
h2 = Dense(100, activation='relu', name='h2')(h1)
h2 = BatchNormalization()(h2)
V = Dense(1, name='V', activation='linear')(h2)

# 1 action
P = Lambda(lambda x: K.square(K.exp(x)), name='P')(Dense(1, activation='linear')(h2))
mu = Lambda(lambda x: 3*x, name='mu')(Dense(1, activation='tanh')(h2))
mu_neg = Lambda(lambda x: -x)(mu)
mu_rel = Add()([a, mu_neg])
mu_rel_sq = Lambda(lambda x: -0.5*K.square(x))(mu_rel)
A = Multiply(name='A')([P, mu_rel_sq])
Q = Add(name='Q')([V, A])

model = Model(input=inp, output=Q)
model.summary()

model.compile(loss='mse', optimizer=Adam(lr=0.001))

# Must be in this order to enable automatic discovery
model._make_train_function()
model._make_predict_function()

# Make sure weight assign placeholders are created
weights = model.get_weights()
model.set_weights(weights)

tf.train.write_graph(get_session().graph.as_graph_def(), './', 'pendulum_naf.pb', as_text=False)

#writer = tf.summary.FileWriter("/tmp/tensorflow", get_session().graph)
#
#data = np.genfromtxt('transitions.csv', delimiter=',')
#x = data[:, 4:7]
#y = data[:, 7:8]
#
#print np.concatenate((x, y), axis=1)
#
#_info = K.function([K.learning_phase(), inp], [s, a, mu, V, Q, P])
#info = lambda x: _info([0] + [x])
#
#np.set_printoptions(threshold=np.nan)
#for i in range(1, 50):
#  print (i)
#  indexes = np.random.choice(x.shape[0], size=100)
#  model.train_on_batch(x[indexes], y[indexes])
#  print(np.hstack((np.hstack(info(x)), y))[0:10,:])
#
#writer.close()
