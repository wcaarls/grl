% cd grl/build
% addpath ../bin
% grl_test

grl_env('init', '../cfg/matlab/pendulum_swingup.yaml');
grl_agent('init', '../cfg/matlab/sarsa.yaml');

o = grl_env('start');
a = grl_agent('start', o);

t = 0
while t == 0
  [o, r, t] = grl_env('step', a);
  a = grl_agent('step', r, o);
  pause(0.05)
end

grl_env('fini')
grl_agent('fini');
