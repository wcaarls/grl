#include <grl/environments/pendulum.h>

using namespace grl;

REGISTER_CONFIGURABLE(PendulumDynamics)
REGISTER_CONFIGURABLE(PendulumSwingupTask)

void PendulumDynamics::request(ConfigurationRequest *config)
{
}

void PendulumDynamics::configure(const Configuration &config)
{
  J_ = 0.000191;
  m_ = 0.055;
  g_ = 9.81;
  l_ = 0.042;
  b_ = 0.000003;
  K_ = 0.0536;
  R_ = 9.5;
}

void PendulumDynamics::reconfigure(const Configuration &config)
{
}

PendulumDynamics *PendulumDynamics::clone() const
{
  return new PendulumDynamics(*this);
}

void PendulumDynamics::eom(const Vector &state, const Vector &action, Vector *xd) const
{
  double a   = state[0];
  double ad  = state[1];
  double add = (1/J_)*(m_*g_*l_*sin(a)-b_*ad-(K_*K_/R_)*ad+(K_/R_)*action[0]);
  
  xd->resize(3);
  (*xd)[0] = ad;
  (*xd)[1] = add;
  (*xd)[2] = 1;
}

void PendulumSwingupTask::request(ConfigurationRequest *config)
{
}

void PendulumSwingupTask::configure(const Configuration &config)
{
  T_ = 2.99;
}

void PendulumSwingupTask::reconfigure(const Configuration &config)
{
}

PendulumSwingupTask *PendulumSwingupTask::clone() const
{
  return new PendulumSwingupTask(*this);
}

void PendulumSwingupTask::start(Vector *state) const
{
  state->resize(3);
  (*state)[0] = M_PI;
  (*state)[1] = 0;
  (*state)[2] = 0;
}

void PendulumSwingupTask::observe(const Vector &state, Vector *obs, int *terminal) const
{
  double a = fmod(state[0]+M_PI, 2*M_PI);
  if (a < 0) a += 2*M_PI;
  
  obs->resize(2);
  (*obs)[0] = a;
  (*obs)[1] = state[1];
  if (state[2] > T_)
    *terminal = 1;
  else
    *terminal = 0;
}

bool PendulumSwingupTask::evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const
{
  double a = fmod(next[0], 2*M_PI);

  *reward = -5*pow(a, 2) - 0.1*pow(next[1], 2) - 1*pow(action[0], 2);
  
  return true;
}
