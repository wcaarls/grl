--[[
--  Cart-pole swing-up task for Manuel's RBDL cart-pole simulation
--  using MPRL-style observations and rewards.
--
--  Authors:
--    Wouter Caarls <wouter@caarls.org>
--    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
--]]

-- Helper functions
math.randomseed( os.time() )

-- Default parameters
if (shaping_weight == nil) then
    -- if shaping_weight is not defined, controls reqularization is not used
    shaping_weight = 0
end

-- Set the following values to enable reward shaping
reward_shaping = true
shaping_gamma = 1.00
cart_pos_max = 2.4

-- Set the following value to enable termination on swingup
terminate_on_swingup = false

function getPotentialSquared(state)
  return -2    *  state[0]^2 
         -1    *  state[1]^2
         -0.2  *  state[2]^2
         -0.5  *  state[3]^2
end

function getPotentialAbsolute(state)
  return -2    *  math.abs(state[0]) 
         -1    *  math.abs(state[1])
         -0.2  *  math.abs(state[2])
         -0.5  *  math.abs(state[3])
end

function failed(state)
  if state[0] < -cart_pos_max or state[0] > cart_pos_max then
    return true
  else
    return false
  end
end

function succeeded(state)
  if state[0] < 0.1                      and  state[0] > -0.1                   and
     state[1] < math.pi + 5*math.pi/180  and  state[1] > math.pi-5*math.pi/180  and
     state[2] < 0.5                      and  state[2] > -0.5                   and
     state[3] < 50*math.pi/180           and  state[3] > -50*math.pi/180        then
    return true
  else
    return false
  end
end

function int(v)
  if v then
    return 1
  else
    return 0
  end
end

-- Exported functions

function configure(argstr)
  T = 5
  return {observation_dims = 4,
          observation_min = {-cart_pos_max, -5*math.pi, -5, -10*math.pi},
          observation_max = { cart_pos_max,  5*math.pi,  5,  10*math.pi},
          action_dims = 1,
          action_min = {-150},
          action_max = { 150},
          reward_min = -10000,
          reward_max = 0
          }
end

function start(test)
  if test == 1 then -- !!! Lua always treats 'numbers' as 'true'. For a correct boolean expression compare numbers !!! --
    return {0, math.pi, 0, 0, 0}
  else
    return {0, math.pi+math.random()*0.1-0.05, 0, 0, 0}
  end
end

function observe(state)
  if failed(state) or (succeeded(state) and terminate_on_swingup) then
    t = 2
  elseif state[4] > T - 1E-3 then
    t = 1
  else
    t = 0
  end
  
  return {state[0], state[1], state[2], state[3]}, t
end

function evaluate(state, action, next)
  if failed(next) then
    if not reward_shaping then
      return -1000
    else
      return -1000
    end
  else
    if not reward_shaping then
      return getPotentialSquared(next) - 0.0005*action[0]^2
    else
      return getPotentialSquared(next) - 0.0005*action[0]^2 + shaping_weight*(shaping_gamma * getPotentialAbsolute(next) - getPotentialAbsolute(state))
    end
  end
end

function invert(obs)
  return {obs[0], obs[1], obs[2], obs[3], 0}
end
