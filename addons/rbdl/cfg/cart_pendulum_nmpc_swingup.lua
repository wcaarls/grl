--[[
--  Cart-pole swing-up task for Manuel's RBDL cart-pole swimulation
--
--  Author:
--    Wouter Caarls <wouter@caarls.org>
--]]

function configure(timeout)
  T = 3
  return {observation_dims = 4,
          observation_min = {-2.4, -2*math.pi, -24, -20*math.pi},
          observation_max = { 2.4,  2*math.pi,  24,  20*math.pi},
          action_dims = 2,
          action_min = {-150, 0},
          action_max = {150, 0},
          reward_min = 0,
          reward_max = 0
          }
end

function start()
  return {0, 3.14, 0, 0, 0}
end

function observe(state)
  if state[4] > T then
    t = 1
  else
    t = 0
  end
  
  return {state[0], state[1], state[2], state[3]}, t
end

function evaluate(state, action, next)
  return 0
end

function invert(obs)
  return {obs[0], obs[1], obs[2], obs[3], 0}
end
