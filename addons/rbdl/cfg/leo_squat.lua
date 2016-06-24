--[[
--  Squatting task for RBDL Leo simulation
--  using MPRL-style observations and rewards.
--
--  Authors:
--    Wouter Caarls <wouter@caarls.org>
--    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
--]]

-- Helper functions
math.randomseed( os.time() )
pi = math.pi

maxvoltage = 10.7

function failed(state)
  if state[0] < -cart_pos_max or state[0] > cart_pos_max then
    return true
  else
    return false
  end
end

function succeeded(state)
  return true
end

-- Exported functions

function configure(argstr)
  T = 10
  return {observation_dims = 20,
          
          observation_min = {-pi, -10*pi, -pi, -10*pi, -pi, -10*pi, -pi, -10*pi, -pi, -10*pi, -pi, -10*pi, -pi, -10*pi, -pi, -10*pi, 0, 0, 0, 0},
          observation_max = { pi,  10*pi,  pi,  10*pi,  pi,  10*pi,  pi,  10*pi,  pi,  10*pi,  pi,  10*pi,  pi,  10*pi,  pi,  10*pi, 1, 1, 1, 1},
          action_dims = 7,
          action_min = {-maxvoltage, -maxvoltage, -maxvoltage, -maxvoltage, -maxvoltage, -maxvoltage, -maxvoltage},
          action_max = { maxvoltage,  maxvoltage,  maxvoltage,  maxvoltage,  maxvoltage,  maxvoltage,  maxvoltage},
          reward_min = -1000,
          reward_max = 2000
          }
end

function start(test)
  if test == 1 then -- !!! Lua always treats 'numbers' as 'true'. For a correct boolean expression compare numbers !!! --
    return {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1}
  else
    C = 1*0.087263889; -- 0.087263889 = +/- 5 deg
    r1 = math.random(-C, C);
    r2 = math.random(-C, C);
    r3 = math.random(-C, C);
    return {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1}
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
  
  return {state[0], wrap_angle(state[1]), state[2], state[3]}, t
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
  return {obs[0], obs[1]-math.pi, obs[2], obs[3], 0}
end
