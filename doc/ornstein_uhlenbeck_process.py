import matplotlib.pyplot as plt
import numpy as np
import math

l = 50
lim = np.array([-7, 7])
theta = 0.15
sigma = 2

x = np.arange(l)
y = np.zeros(l)

for i in x[1:]:
  y[i] = y[i-1]*(1-theta) + sigma*np.random.normal(0, 1)
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
