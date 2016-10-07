import matplotlib.pyplot as plt
import numpy as np
import math
from mpl_toolkits.mplot3d import axes3d
from matplotlib import cm

l = 500
y0 = 0
discrete = 7

def main():
  lim = np.array([-10.7, 10.7])

  if 1:
    sigma = 3.0
    theta = 0.57
    plot_single(sigma, theta, lim)
  else:
    plot_grid(np.linspace(0, 3, 10), np.linspace(0, 1, 10), lim)

def plot_grid(sigma, theta, lim):
  x = np.arange(l)
  
  w, h = len(sigma), len(theta)
  Z = [[0 for i in range(w)] for j in range(h)] 
  
  for i, s in enumerate(sigma):
    for j, t in enumerate(theta):
      a, crossing_proc, crossing_mag = get_ou_data(s, t, lim)
      Z[j][i] = crossing_mag
      #print '{:04f} {:04f} {}'.format(s, t, crossing_mag)

  X, Y = np.meshgrid(sigma, theta)
  #print X, Y
  fig = plt.figure()
  if 0:
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(X, Y, np.matrix(Z), rstride=1, cstride=1, cmap=cm.coolwarm,
                         linewidth=0, antialiased=False)

    ax.set_zlim3d(np.matrix.min(np.matrix(Z)), np.matrix.max(np.matrix(Z)))
    fig.colorbar(surf, shrink=0.5, aspect=10)
  else:
    levels = np.linspace(np.matrix.min(np.matrix(Z)), np.matrix.max(np.matrix(Z)), 20)
    CS = plt.contour(X, Y, Z, levels=levels, cmap=cm.winter)
    plt.clabel(CS, inline=1, fontsize=10)
    plt.grid(True)
    plt.xticks(np.linspace(0, 3, 16))
    plt.yticks(np.linspace(0, 1, 11))

  plt.title('Magnitude')
  ax = fig.gca()
  ax.set_xlabel('sigma')
  ax.set_ylabel('theta')
  plt.show()

def plot_single(sigma, theta, lim):
  x = np.arange(l)
  a, crossing_proc, crossing_mag = get_ou_data(sigma, theta, lim)

  print 'Crossing zero {} percent, magnitude is {}'.format(crossing_proc, crossing_mag)

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


def get_ou_data(sigma, theta, lim):
  """Calculate MTBF of the given data"""

  if discrete:
    step = (lim[1] - lim[0]) / (discrete-1)
  #print step

  x = np.arange(l)
  y = np.zeros(l)
  a = np.zeros(l)

  for i in x[1:]:
    y[i] = y[i-1] + theta*(y0 - y[i-1]) + sigma * np.random.normal(0, 1)

    if (y[i] < lim[0]):
      y[i] = lim[0]
    if (y[i] > lim[1]):
      y[i] = lim[1]
    
    if discrete:
      y[i] = np.trunc(y[i]/step + np.copysign(0.5,y[i])) * step

  if discrete:
    a = np.trunc(y/step + np.copysign(0.5,y))
  else:
    a = y

  Tm = []
  prev_prev = 0;
  prev = a[0]
  for i in np.arange(1, l):
    curr = a[i]
    if ((prev <= 0 and curr > 0) or (prev >= 0 and curr < 0)):
      if (prev == 0 and prev_prev*curr < 0 or prev != 0):
        Tm.append(curr)
    prev_prev = prev;
    prev = curr;

  Tm = np.asarray(Tm)
  Tm = np.absolute(Tm)

  return a, np.count_nonzero(Tm)/float(l), np.sum(Tm)/float(l)

if __name__ == "__main__":
  main()

