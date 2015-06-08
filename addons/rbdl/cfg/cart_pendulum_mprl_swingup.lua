--[[
--  Cart-pole swing-up task for Manuel's RBDL cart-pole simulation
--  using MPRL-style observations and rewards.
--
--  Author:
--    Wouter Caarls <wouter@caarls.org>
--    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
--]]

-- Helper functions

function wrap_angle(a)
  a = a%(2*math.pi)  
  if a < math.pi then
    a = a + math.pi -- [  0; pi) -> [pi; 2pi)
  else
    a = a - math.pi -- [2pi; pi] -> [pi; 0]
  end
  return a
end

-- Angle is changes so that it is 0 at swingup, and -pi or pi in the bootom.
-- This ensures that maximum potential is 0 (at swingup) and minimum is < 0 everywhere else
function getPotential(state)
  angle = wrap_angle(state[1]) - math.pi
  return -1    *  angle^2
         -0.1  *  state[3]^2
         -2    *  state[0]^2 
         -0.1  *  state[2]^2
end

function failed(state)
  if state[0] < -2.4 or state[0] > 2.4 then
    return true
  else
    return false
  end
end

-- Exported functions

function configure(argstr)
  T = 10
  return {observation_dims = 4,
          observation_min = {-2.4, 0,         -5, -10*math.pi},
          observation_max = { 2.4, 2*math.pi,  5,  10*math.pi},
          action_dims = 1,
          action_min = {-15},
          action_max = {15},
          reward_min = -10000,
          reward_max = 0
          }
end

function start()
  return {0, 3.14+math.random()*0.1-0.05, 0, 0, 0}
end

function observe(state)
  if failed(state) then
    t = 2
  elseif state[4] > T then
    t = 1
  else
    t = 0
  end
  
  return {state[0], wrap_angle(state[1]), state[2], state[3]}, t
end

function evaluate(state, action, next)
  if failed(next) then
    return -10000
  else
    return getPotential(next)
  end
end

function invert(obs)
  return {obs[0], obs[1]-math.pi, obs[2], obs[3], 0}
end
