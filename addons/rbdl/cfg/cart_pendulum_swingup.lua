--[[
--  Cart-pole swing-up task for Manuel's RBDL cart-pole simulation
--  using MPRL-style observations and rewards.
--
--  Author:
--    Wouter Caarls <wouter@caarls.org>
--    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
--]]

-- Helper functions

-- Set the following values to enable reward shaping
reward_shaping = false
shaping_gamma = 1.00

-- Set the following value to enable termination on swingup
terminate_on_swingup = false

function wrap_angle(a)
  a = a%(2*math.pi)  
  if a < math.pi then
    a = a + math.pi -- [  0; pi) -> [pi; 2pi)
  else
    a = a - math.pi -- [2pi; pi] -> [pi; 0]
  end
  return a
end

-- Angle is changed so that it is 0 at swingup, and -pi or pi in the bootom.
-- This ensures that maximum potential is 0 (at swingup) and minimum is < 0 everywhere else
function getPotential(state)
  angle = wrap_angle(state[1]) - math.pi

-- Type 1:
--  return -1    *  state[0]^2 
--         -10   *  angle^2
--         -0.1  *  state[2]^2 
--         -0.1  *  state[3]^2

--  Type 2:
  return -2    *  state[0]^2 
         -1    *  angle^2
         -0.1  *  state[2]^2 
         -0.5  *  state[3]^2
end

function failed(state)
  if state[0] < -2.4 or state[0] > 2.4 then
    return true
  else
    return false
  end
end

function succeeded(state)
  angle = wrap_angle(state[1])
  if state[0] < 0.1                      and  state[0] > -0.1                   and
     angle    < math.pi + 5*math.pi/180  and  angle    > math.pi-5*math.pi/180  and
     state[2] < 0.5                      and  state[2] > -0.5                   and
     state[3] < 50*math.pi/180           and  state[3] > -50*math.pi/180        then
    return true
  else
    return false
  end
end

-- Exported functions

function configure(argstr)
  T = 3
  return {observation_dims = 4,
          observation_min = {-2.4, 0,         -5, -10*math.pi},
          observation_max = { 2.4, 2*math.pi,  5,  10*math.pi},
          action_dims = 1,
          action_min = {-150},
          action_max = {150},
          reward_min = -10000,
          reward_max = 0
          }
end

function start()
  return {0, math.pi, 0, 0, 0}
end

function observe(state)
  if failed(state) or (succeeded(state) and terminate_on_swingup) then
    t = 2
  elseif state[4] > T - 1E-3 then
    t = 1
  else
    t = 0
  end
  
  return {state[0], wrap_angle(state[1]), state[2], state[3]}, t
end

function evaluate(state, action, next)
  if failed(next) then
    return -1000
  else
    return getPotential(next)
  end

  if failed(next) then
    if not reward_shaping then
      return -1000
    else
      return -100
    end
  else
    if not reward_shaping then
      return getPotential(next)
    else
      reward = shaping_gamma * getPotential(next) - getPotential(state)
      if succeeded(next) then
        reward = reward + 10
      end
      return reward
    end
  end
end

function invert(obs)
  return {obs[0], obs[1]-math.pi, obs[2], obs[3], 0}
end
