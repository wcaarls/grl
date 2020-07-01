/** \file quadcopter.cpp
 * \brief Quadcopter dynamics source file
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2020-04-14
 *
 * \copyright \verbatim
 * Copyright(c) 2017 Abhijit Majumdar
 * Copyright(c) 2020 Wouter Caarls
 *
 * Based on
 * https://github.com/abhijitmajumdar/Quadcopter_simulator/blob/master/quadcopter.py
 * which carries the following license:
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files(the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions :
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <grl/environments/quadcopter.h>

using namespace grl;
using namespace Eigen;

REGISTER_CONFIGURABLE(QuadcopterDynamics)
REGISTER_CONFIGURABLE(QuadcopterRegulatorTask)
REGISTER_CONFIGURABLE(QuadcopterRateController)
REGISTER_CONFIGURABLE(QuadcopterAttitudeController)
REGISTER_CONFIGURABLE(QuadcopterVelocityController)
REGISTER_CONFIGURABLE(QuadcopterPositionController)

void QuadcopterDynamics::request(ConfigurationRequest *config)
{
  config->push_back(CRP("g", "Gravitational constant", g_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("b", "Drag coefficient", b_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("L", "Propeller distance from center", L_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("r", "Central body radius", r_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("M", "Central body weight", weight_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("d", "Propeller diameter", prop_dia_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("p", "Propeller pitch", prop_pitch_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("m", "Motor weight", prop_weight_, CRP::Configuration, 0., DBL_MAX));

  config->push_back(CRP("limits", "vector.limits/quadcopter", "Position and velocity limits (0=unlimited)", limits_, CRP::Configuration));
}

void QuadcopterDynamics::configure(Configuration &config)
{
  g_ = config["g"];
  b_ = config["b"];
  L_ = config["L"];
  r_ = config["r"];
  weight_ = config["M"];
  prop_dia_ = config["d"];
  prop_pitch_ = config["p"];
  prop_weight_ = config["m"];

  limits_ = config["limits"].v();
  if (limits_.size() != 2)
    throw bad_param("dynamics/quadcopter:limits");
  
  for (size_t ii=0; ii != 4; ++ii)
    propellers_.push_back(Propeller(prop_dia_, prop_pitch_));

  // From Quadrotor Dynamics and Control by Randal Beard
  double ixx = ((2 * weight_ * r_ * r_) / 5) + (2 * prop_weight_ * L_ * L_);
  double iyy = ixx;
  double izz = ((2 * weight_ * r_ * r_) / 5) + (4 * prop_weight_ * L_ * L_);

  I_ << ixx, 0, 0, 0, iyy, 0, 0, 0, izz;
  invI_ = I_.inverse();
}

void QuadcopterDynamics::reconfigure(const Configuration &config)
{
}

void QuadcopterDynamics::eom(const Vector &state, const Vector &actuation, Vector *xd) const
{
  // State space representation : [x y z x_dot y_dot z_dot theta phi gamma theta_dot phi_dot gamma_dot]
  // Action space representation: [left, front, right, back]
  // From Quadcopter Dynamics, Simulation, and Control by Andrew Gibiansky
  if (state.size() != 13 || actuation.size() != 4)
    throw Exception("dynamics/quadcopter requires a task/quadcopter subclass");

  double t[4];
  for (size_t ii = 0; ii != 4; ++ii)
    t[ii] = propellers_[ii].getThrust(actuation[ii]);
    
  xd->resize(13);

  // The velocities(t + 1 x_dots equal the t x_dots)
  (*xd)[0] = state[3];
  (*xd)[1] = state[4];
  (*xd)[2] = state[5];

  // The acceleration
  Vector3d x_dotdot = Vector3d(0, 0, -weight_*g_) +
                      rotationMatrix(state[6], state[7], state[8]) * Vector3d(0, 0, t[0]+t[1]+t[2]+t[3]) / weight_;
  (*xd)[3] = x_dotdot[0];
  (*xd)[4] = x_dotdot[1];
  (*xd)[5] = x_dotdot[2];

  // The angular rates(t + 1 theta_dots equal the t theta_dots)
  (*xd)[6] = state[9];
  (*xd)[7] = state[10];
  (*xd)[8] = state[11];

  // The angular accelerations (small angle approximation for omega)
  Vector3d omega = Vector3d(state[9], state[10], state[11]);
  Vector3d tau = Vector3d(L_ * (t[0] - t[2]), L_ * (t[1] - t[3]), b_ * (t[0] - t[1] + t[2] - t[3]));
  Vector3d omega_dot = invI_ * (tau - omega.cross(I_*omega));

  (*xd)[9] = omega_dot[0];
  (*xd)[10] = omega_dot[1];
  (*xd)[11] = omega_dot[2];

  // Time
  (*xd)[12] = 1;
  
  // Limit position
  if (limits_[0])
    for (size_t ii=0; ii != 3; ++ii)
    {
      if (state[ii] > limits_[0])
      {
        if ((*xd)[0+ii] > 0) (*xd)[0+ii] = 0;
        if ((*xd)[3+ii] > 0) (*xd)[3+ii] = 0;
      }
      if (state[ii] < -limits_[0])
      {
        if ((*xd)[0+ii] < 0) (*xd)[0+ii] = 0;
        if ((*xd)[3+ii] < 0) (*xd)[3+ii] = 0;
      }
    }

  // Limit velocities
  if (limits_[1])
    for (size_t ii=0; ii != 3; ++ii)
    {
      if ((*xd)[ii] > limits_[1])
      {
        (*xd)[ii] = limits_[1];
        if ((*xd)[3+ii] > 0) (*xd)[3+ii] = 0;
      }
      if ((*xd)[ii] < -limits_[1])
      {
        (*xd)[ii] = -limits_[1];
        if ((*xd)[3+ii] < 0) (*xd)[3+ii] = 0;
      }
    }
}

Matrix3d QuadcopterDynamics::rotationMatrix(double theta, double phi, double gamma) const
{
  double ct = cos(theta);
  double cp = cos(phi);
  double cg = cos(gamma);
  double st = sin(theta);
  double sp = sin(phi);
  double sg = sin(gamma);
  Matrix3d R_x, R_y, R_z;

  // ZYX (321) form  
  R_x << 1, 0, 0, 0, ct, -st, 0, st, ct;
  R_y << cp, 0, sp, 0, 1, 0, -sp, 0, cp;
  R_z << cg, -sg, 0, sg, cg, 0, 0, 0, 1;
  
  return R_z * (R_y * R_x);
}

// Regulator

void QuadcopterRegulatorTask::request(ConfigurationRequest *config)
{
  RegulatorTask::request(config);
  
  config->push_back(CRP("action_range", "Range of allowed actions (rpm)", action_range_, CRP::Configuration));
  config->push_back(CRP("limits", "vector.limts/quadcopter", "Position and velocity limits (0=unlimited)", limits_, CRP::Configuration));
  config->push_back(CRP("wrap", "Wrap positions around limits", wrap_, CRP::Configuration, 0, 1));
  config->push_back(CRP("time_reward", "Reward per second", time_reward_, CRP::Configuration, 0., DBL_MAX));
  config->push_back(CRP("limit_penalty", "Terminal penalty for crossing position limit", limit_penalty_, CRP::Configuration, 0., DBL_MAX));
  
  config->push_back(CRP("controller", "controller/quadcopter", "Downstream controller to convert action into propeller rpm", controller_, true));
}

void QuadcopterRegulatorTask::configure(Configuration &config)
{
  RegulatorTask::configure(config);
  
  action_range_ = config["action_range"].v();
  if (action_range_.size() == 1)
    action_range_ = VectorConstructor(0, action_range_[0]);
  if (action_range_.size() != 2)
    throw bad_param("task/quadcopter/regulator:action_range");
    
  limits_ = config["limits"].v();
  if (limits_.size() != 2)
    throw bad_param("dynamics/quadcopter:limits");
    
  wrap_ = config["wrap"];
  time_reward_ = config["time_reward"];
  limit_penalty_ = config["limit_penalty"];
  
  controller_ = (Controller*)config["controller"].ptr();
  size_t action_dims = 4;
  if (controller_)
    action_dims = (*controller_->configurator())["action_dims"];
  
  if (wrap_ && !limits_[0])
  {
    ERROR("Cannot wrap without valid position limit");
    throw bad_param("dynamics/quadcopter:{limits,wrap}");
  }
  
  if (q_.size() != 12)
    throw bad_param("task/quadcopter/regulator:q");
  if (r_.size() != action_dims)
    throw bad_param("task/quadcopter/regulator:r");
    
  double p = limits_[0]?limits_[0]:1, v = limits_[1]?limits_[1]:10;

  config.set("observation_min", VectorConstructor(-p, -p, -p, -M_PI, -M_PI, -M_PI, -v, -v, -v, -10*M_PI, -10*M_PI, -10*M_PI));
  config.set("observation_max", VectorConstructor( p,  p,  p,  M_PI,  M_PI,  M_PI,  v,  v,  v,  10*M_PI,  10*M_PI,  10*M_PI));
  
  if (controller_)
  {
    config.set("action_min", (*controller_->configurator())["action_min"].v());
    config.set("action_max", (*controller_->configurator())["action_max"].v());
  }
  else
  {
    config.set("action_min", VectorConstructor(0, 0, 0, 0));
    config.set("action_max", VectorConstructor(1, 1, 1, 1));
  }
}

void QuadcopterRegulatorTask::reconfigure(const Configuration &config)
{
  RegulatorTask::reconfigure(config);
}

bool QuadcopterRegulatorTask::actuate(const Vector &prev, const Vector &state, const Action &action, Vector *actuation) const
{
  bool retval = true;

  if (controller_)
    retval = controller_->actuate(state, action, actuation);
  else
    *actuation = action;
    
  // Convert to minrpm-maxrpm
  for (size_t ii=0; ii != actuation->size(); ++ii)
    (*actuation)[ii] = clip((*actuation)[ii]*(action_range_[1]-action_range_[0])+action_range_[0], action_range_[0], action_range_[1]);
    
  return retval;
}

void QuadcopterRegulatorTask::observe(const Vector &state, Observation *obs, int *terminal) const
{
  RegulatorTask::observe(state, obs, terminal);

  if (state.size() != 13)
    throw Exception("task/quadcopter/regulator requires dynamics/quadcopter");

  Vector _state = wrap(state);
    
  obs->v.resize(12);
  for (size_t ii=0; ii < 12; ++ii)
    (*obs)[ii] = _state[ii];
  
  if (failed(state))
  {
    obs->absorbing = true;
    *terminal = 2;
  }
  else
    obs->absorbing = false;
}

void QuadcopterRegulatorTask::evaluate(const Vector &state, const Action &action, const Vector &next, double *reward) const
{
  // Bound errors
  Vector _state = wrap(state), _next = wrap(next);
  
  RegulatorTask::evaluate(_state, action, _next, reward);
  
  *reward += time_reward_;
  
  if (failed(next))
    *reward -= limit_penalty_;
}

bool QuadcopterRegulatorTask::invert(const Observation &obs, Vector *state, double time) const
{
  *state = extend(obs, VectorConstructor(time));
  
  return true;
}

Vector QuadcopterRegulatorTask::wrap(const Vector &state) const
{
  Vector _state = state;

  for (size_t ii=0; ii != 3; ++ii)
  {    
    // Simulate 3D torus
    if (wrap_)
    {
      double p = fmod(state[ii]+limits_[0], 2*limits_[0]);
      if (p < 0) p += 2*limits_[0];
      p -= limits_[0];
      _state[ii] = p;
    }

    // Bound angles between -pi, pi
    double a = fmod(state[3+ii]+M_PI, 2*M_PI);
    if (a < 0) a += 2*M_PI;
    a -= M_PI;
    _state[3+ii] = a;
  }  
  
  return _state;
}

bool QuadcopterRegulatorTask::failed(const Vector &state) const
{
  if (limit_penalty_ && (fabs(state[0]) > limits_[0] || 
                         fabs(state[1]) > limits_[0] ||
                         fabs(state[2]) > limits_[0]))
    return true;
  else
    return false;
}

// QuadcopterRateController

void QuadcopterRateController::configure(Configuration &config)
{
  config.set("action_dims", 4);
  config.set("action_min", VectorConstructor(-1, -1, -1, 0));
  config.set("action_max", VectorConstructor( 1,  1,  1, 1));
}

bool QuadcopterRateController::actuate(const Vector &state, const Action &action, Vector *actuation) const
{
  if (action.size() != 4)
    throw Exception("controller/quadcopter/rate requires task/quadcopter");

  Matrix delta(4, 4);
//  delta <<  1,  1, -1, 1,
//           -1,  1,  1, 1,
//           -1, -1, -1, 1,
//            1, -1,  1, 1;
  delta <<  1,  0,  1, 1,
            0,  1, -1, 1,
           -1,  0,  1, 1,
            0, -1, -1, 1;

  Vector clipped = VectorConstructor(clip(action[0], -1., 1.),
                                     clip(action[1], -1., 1.),
                                     clip(action[2], -1., 1.),
                                     clip(action[3],  0., 1.));
           
  *actuation = delta*clipped.matrix().transpose();
  
  return true;
}

// QuadcopterAttitudeController

void QuadcopterAttitudeController::request(ConfigurationRequest *config)
{
  QuadcopterRateController::request(config);
  
  config->push_back(CRP("p_att", "P gains (roll, pitch, yaw_rate, climb_rate)", p_, CRP::Configuration));
  config->push_back(CRP("d_att", "D gains (roll, pitch)", d_, CRP::Configuration));
  config->push_back(CRP("ff_att", "Feedforward gains (climb_rate)", ff_, CRP::Configuration));
}

void QuadcopterAttitudeController::configure(Configuration &config)
{
  QuadcopterRateController::configure(config);

  p_ = config["p_att"].v();
  d_ = config["d_att"].v();
  ff_ = config["ff_att"].v();
  
  if (p_.size() != 4)
    throw bad_param("controller/quadcopter/attitude:p_att");
    
  if (d_.size() != 2)
    throw bad_param("controller/quadcopter/attitude:d_att");

  if (ff_.size() != 1)
    throw bad_param("controller/quadcopter/attitude:ff_att");

  config.set("action_dims", 4);
  config.set("action_min", VectorConstructor(-0.5*M_PI, -0.5*M_PI, -2*M_PI, -1));
  config.set("action_max", VectorConstructor( 0.5*M_PI,  0.5*M_PI,  2*M_PI,  1));
}

bool QuadcopterAttitudeController::actuate(const Vector &state, const Action &action, Vector *actuation) const
{
  if (state.size() != 13 || action.size() != 4)
    throw Exception("controller/quadcopter/attitude requires task/quadcopter");

  Vector clipped = VectorConstructor(clip(action[0], -0.5*M_PI, 0.5*M_PI),
                                     clip(action[1], -0.5*M_PI, 0.5*M_PI),
                                     clip(action[2], -2.0*M_PI, 2.0*M_PI),
                                     clip(action[3], -1.0     , 1.0     ));
           
  Vector pact = p_ * (clipped - VectorConstructor(state[6], state[7], state[11], state[5]));
  Vector dact = d_ * -VectorConstructor(state[9], state[10]);
  
  Vector downstream_action = pact + extend(dact, VectorConstructor(0., ff_[0]));
  return QuadcopterRateController::actuate(state, downstream_action, actuation);
}

// QuadcopterVelocityController

void QuadcopterVelocityController::request(ConfigurationRequest *config)
{
  QuadcopterAttitudeController::request(config);
  
  config->push_back(CRP("p_vel", "P gains (x_vel, y_vel)", p_, CRP::Configuration));
}

void QuadcopterVelocityController::configure(Configuration &config)
{
  QuadcopterAttitudeController::configure(config);

  p_ = config["p_vel"].v();
  
  if (p_.size() != 2)
    throw bad_param("controller/quadcopter/velocity:p_vel");

  config.set("action_dims", 4);
  config.set("action_min", VectorConstructor(-1, -1, -1, -2*M_PI));
  config.set("action_max", VectorConstructor( 1,  1,  1,  2*M_PI));
}

bool QuadcopterVelocityController::actuate(const Vector &state, const Action &action, Vector *actuation) const
{
  if (state.size() != 13 || action.size() != 4)
    throw Exception("controller/quadcopter/velocity requires task/quadcopter");

  Vector clipped = VectorConstructor(clip(action[0], -1.0     , 1.0     ),
                                     clip(action[1], -1.0     , 1.0     ),
                                     clip(action[2], -1.0     , 1.0     ),
                                     clip(action[3], -2.0*M_PI, 2.0*M_PI));
           
  Vector pact = p_ * (clipped.segment(0, 2) - VectorConstructor(state[3], state[4]));
  
  Vector downstream_action = VectorConstructor(-pact[1], pact[0], action[3], action[2]);
  return QuadcopterAttitudeController::actuate(state, downstream_action, actuation);
}

// QuadcopterPositionController

void QuadcopterPositionController::request(ConfigurationRequest *config)
{
  QuadcopterAttitudeController::request(config);
  
  config->push_back(CRP("p_pos", "P gains (x, y, z, yaw)", p_, CRP::Configuration));
  config->push_back(CRP("d_pos", "D gains (x, y)", d_, CRP::Configuration));
}

void QuadcopterPositionController::configure(Configuration &config)
{
  QuadcopterAttitudeController::configure(config);

  p_ = config["p_pos"].v();
  d_ = config["d_pos"].v();
  
  if (p_.size() != 4)
    throw bad_param("controller/quadcopter/position:p_pos");

  if (d_.size() != 2)
    throw bad_param("controller/quadcopter/position:d_pos");

  config.set("action_dims", 4);
  config.set("action_min", VectorConstructor(-1, -1, -1, -M_PI));
  config.set("action_max", VectorConstructor( 1,  1,  1,  M_PI));
}

bool QuadcopterPositionController::actuate(const Vector &state, const Action &action, Vector *actuation) const
{
  if (state.size() != 13 || action.size() != 4)
    throw Exception("controller/quadcopter/position requires task/quadcopter");

  Vector clipped = VectorConstructor(clip(action[0], -1.0     , 1.0     ),
                                     clip(action[1], -1.0     , 1.0     ),
                                     clip(action[2], -1.0     , 1.0     ),
                                     clip(action[3], -1.0*M_PI, 1.0*M_PI));
           
  Vector pact = p_ * (clipped - VectorConstructor(state[0], state[1], state[2], state[8]));
  Vector dact = d_ * -VectorConstructor(state[3], state[4]);
  
  Vector downstream_action = VectorConstructor(-pact[1]-dact[1], pact[0]+dact[0], pact[3], pact[2]);
  return QuadcopterAttitudeController::actuate(state, downstream_action, actuation);
}
