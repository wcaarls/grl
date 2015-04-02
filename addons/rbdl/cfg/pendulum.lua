--[[
--  Pendulum RBDL model
--
--  Author:
--    Wouter Caarls <wouter@caarls.org>
--]]

l = 0.042
m = 0.055
g = 9.81
b = 0.000003
J = 0.000191
K = 0.0536
R = 9.5

wr = 0.02
wd = 0.01
pr = l+wr
pd = 0.001

joints = {
  hinge = {
    { 1., 0., 0., 0., 0., 0.}
  }
}

visuals = {
  pendulum = {
    {
      name = "plate",
      dimensions = { pd, 2*pr, 2*pr },
      color = {0., 0., 1.},
      mesh_center = { 0., 0., 0. },
      src = "meshes/unit_cylinder_medres_z.obj"
    },
    {
      name = "weight",
      dimensions = { wd, 2*wr, 2*wr },
      color = {.8, .8, .8},
      mesh_center = { (pd+wd)/2, 0., l },
      src = "meshes/unit_cylinder_medres_z.obj"
    }
  }
}

-- **************************************************************************
-- *** CONTROLLER ***********************************************************
-- **************************************************************************

function control(state, action)
  return {-(b+K^2/R)*state[1]+(K/R)*action[0]}
end

-- **************************************************************************
-- *** MODEL ****************************************************************
-- **************************************************************************

model = {
  gravity = {0., 0., -g},
  
  frames = {
    {
      name = "pendulum",
      parent = "ROOT",
      body = {
        mass = m,
        com = {0., 0., l},
        inertia = {{J, 0., 0.},
                   {0., J, 0.},
                   {0., 0.0, 0.}}
      },
      joint = joints.hinge,
      visuals = visuals.pendulum
    }
  },
}

return model
