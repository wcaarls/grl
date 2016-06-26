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

reward_shaping = false
terminate_on_success = false
maxvoltage = 10.7

function failed(state)
  if state[0] < -1.4 or state[0] > 1.4 then
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
  if failed(state) or (succeeded(state) and terminate_on_success) then
    t = 2
  elseif state[4] > T - 1E-3 then
    t = 1
  else
    t = 0
  end
  
  return {state[0], state[1], state[2], state[3], state[4], state[5], state[6], state[7], state[8], state[9], state[10], state[11], state[12], state[12], state[14], state[15], state[16], state[17], state[18], state[19]}, t
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
      return 1
    end
  end
end

function invert(obs)
  return {obs[0], obs[1], obs[2], obs[3], obs[4], obs[5], obs[6], obs[7], obs[8], obs[9], obs[10], obs[11], obs[12], obs[12], obs[14], obs[15], obs[16], obs[17], obs[18], obs[19], 0}
end
