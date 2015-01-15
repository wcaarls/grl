#include <grl/environment.h>

using namespace grl;

REGISTER_CONFIGURABLE(ModeledEnvironment)
REGISTER_CONFIGURABLE(DynamicalModel)

void ModeledEnvironment::request(ConfigurationRequest *config)
{
}

void ModeledEnvironment::configure(const Configuration &config)
{
  model_ = (Model*)config["model"].ptr();
  task_ = (Task*)config["task"].ptr();
}

void ModeledEnvironment::reconfigure(const Configuration &config)
{
}
    
ModeledEnvironment *ModeledEnvironment::clone() const
{
  ModeledEnvironment* me = new ModeledEnvironment();
  
  me->model_ = model_;
  me->task_ = task_;
  
  return me;
}

void ModeledEnvironment::start(Vector *obs)
{
  int terminal;

  task_->start(&state_);
  task_->observe(state_, obs, &terminal);
}

void ModeledEnvironment::step(const Vector &action, Vector *obs, double *reward, int *terminal)
{
  Vector next;

  model_->step(state_, action, &next);
  task_->observe(next, obs, terminal);
  task_->evaluate(state_, action, next, reward);
  
  state_ = next;
}

void DynamicalModel::request(ConfigurationRequest *config)
{
}

void DynamicalModel::configure(const Configuration &config)
{
  dynamics_ = (Dynamics*)config["dynamics"].ptr();
  config.get("control_step", tau_, 0.05);
  config.get("integration_steps", steps_, (size_t)5);
}

void DynamicalModel::reconfigure(const Configuration &config)
{
}

DynamicalModel *DynamicalModel::clone() const
{
  DynamicalModel *dm = new DynamicalModel();
  dm->dynamics_ = dynamics_;
  return dm;
}

void DynamicalModel::step(const Vector &state, const Vector &action, Vector *next) const
{
  Vector xd;
  double h = tau_/steps_;
  
  *next = state;
  
  for (size_t ii=0; ii < steps_; ++ii)
  {
    dynamics_->eom(state, action, &xd);
    Vector k1 = h*xd;
    dynamics_->eom(state + k1/2, action, &xd);
    Vector k2 = h*xd;
    dynamics_->eom(state + k2/2, action, &xd);
    Vector k3 = h*xd;
    dynamics_->eom(state + k3, action, &xd);
    Vector k4 = h*xd;

    *next = *next + (k1+2*k2+2*k3+k4)/6;
  }
}
