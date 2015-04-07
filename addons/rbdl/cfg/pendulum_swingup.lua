--[[
--  Pendulum swing-up task
--
--  Author:
--    Wouter Caarls <wouter@caarls.org>
--]]

function configure(timeout)
  T = tonumber(timeout)
  
  return {observation_dims = 2,
          observation_min = {0, -12*math.pi},
          observation_max = {2*math.pi, 12*math.pi},
          action_dims = 1,
          action_min = {-3},
          action_max = {3},
          reward_min = -5*math.pi^2-0.1*(12*math.pi)^2-1*3^2,
          reward_max = 0
          }
end

function start()
  return {math.pi, 0, 0}
end

function observe(state)
  a = (state[0]+math.pi)%(2*math.pi)
  if a < 0 then
    a = a + 2*math.pi
  end
  
  if state[2] > T then
    t = 1
  else
    t = 0
  end
  
  return {a, state[1]}, t
end

function evaluate(state, action, next)
  a = (math.abs(next[0]))%(2*math.pi)
  if a > math.pi then
    a = a - 2*math.pi
  end
  
  return -5*a^2 - 0.1*next[1]^2 - 1*action[0]^2
end

function invert(obs)
  return {obs[0]-math.pi, obs[1], 0}
end
