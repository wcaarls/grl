-- Shorthand
o = grl.new
r = grl.ref 

-- Define configuration. Note that
--   config{a=b, c=d}
-- is equivalent to
--   config.a = b
--   config.c = d
config = o(){
  experiment = o('experiment/online_learning'){
    environment = o('environment/modeled'){
      model = o('model/dynamical'){
        control_step = 0.03,
        integration_steps = 5,
        dynamics = o('dynamics/pendulum')
      },
      task = o('task/pendulum/regulator'){
        start = {0, 0},
        stddev = {0.1, 0.1},
        timeout = 2.99
      }
    },
  }
}

-- Add agent separately, because tables are not ordered and
-- the reference must be defined after the object
config.experiment{
  agent = o('agent/fixed'){
    policy = o('mapping/policy/parameterized/pid'){
      setpoint = {0, 0},
      p = {5, 1},
      action_min = config.experiment.environment.task.action_min,
      action_max = config.experiment.environment.task.action_max,
    }
  }
}

config{
  visualizer = o('visualizer/glut'),
  visualization = o('visualization/pendulum'){
    state = config.experiment.environment.state
  }
}

-- Instantiate configuration and run experiment
config:instantiate().experiment:run()
