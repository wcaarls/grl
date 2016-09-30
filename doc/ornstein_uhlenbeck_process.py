import matplotlib.pyplot as plt
import numpy as np
import math

l = 50
lim = np.array([-10.7, 10.7])

theta = 0.10
sigma = 3
y0 = 0
discrete = 7

x = np.arange(l)
y = np.zeros(l)
a = np.zeros(l)

for i in x[1:]:
  y[i] = y[i-1] + theta*(y0 - y[i-1]) + sigma * np.random.normal(0, 1)

  if (y[i] < lim[0]):
    y[i] = lim[0]
  if (y[i] > lim[1]):
    y[i] = lim[1]

if (discrete):
  a = np.fix(2*y/(discrete-1))
else:
  a = y

plt.plot(x, a)
plt.xlabel('sample')
plt.ylabel('value')
plt.title('Ornstein-Uhlenbeck process for action selection')
plt.grid(True)
axes = plt.gca()
if (discrete):
  lim = np.round(2*lim/(discrete-1))
axes.set_ylim(lim)
plt.show()

