import matplotlib.pyplot as plt
import numpy as np
import math

l = 50

#lim = np.array([0, 6])
#theta = 0.05
#sigma = 1.0 # try with 1.0
#y0 = 3
#discrete = 1

mul = 3.566666667
lim = np.array([-10.7, 10.7])
theta = mul*0.05
sigma = mul*1.0
y0 = 0
discrete = 1

x = np.arange(l)
y = np.zeros(l)

for i in x[1:]:
  y[i] = y[i-1] + theta*(y0 - y[i-1]) + sigma * np.random.normal(0, 1)

  if (discrete):
    y[i] = round(y[i])

  if (y[i] < lim[0]):
    y[i] = lim[0]
  if (y[i] > lim[1]):
    y[i] = lim[1]

plt.plot(x, y)
plt.xlabel('sample')
plt.ylabel('value')
plt.title('Ornstein-Uhlenbeck process for action selection')
plt.grid(True)
axes = plt.gca()
axes.set_ylim(lim)
plt.show()
