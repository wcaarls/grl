
#include <grl/environments/leo/rbdl_leo_model.h>
#include <grl/environments/leo/rbdl_leo_task.h>

using namespace grl;

REGISTER_CONFIGURABLE(LeoSquattingSandboxModel)

void LeoSandboxModel::request(ConfigurationRequest *config)
{
  dm_.request(config);
  config->push_back(CRP("dof_count", "int.dof_count", "Number of degrees of freedom of the model", dof_count_, CRP::Configuration, 0, INT_MAX));
  config->push_back(CRP("target_env", "environment", "Interaction environment", target_env_, true));
  config->push_back(CRP("action_min", "vector.action_min", "Lower limit on actions", action_min_, CRP::System));
  config->push_back(CRP("action_max", "vector.action_max", "Upper limit on actions", action_max_, CRP::System));
}

void LeoSandboxModel::configure(Configuration &config)
{
  dm_.configure(config);

  dof_count_ = config["dof_count"];
  target_env_ = (Environment*)config["target_env"].ptr(); // here we can select a real enviromnent if needed
  action_min_ = config["action_min"].v();
  action_max_ = config["action_max"].v();
}
/*
LeoSandboxModel *LeoSandboxModel::clone() const
{
  LeoSandboxModel *model = new LeoSandboxModel();
  model->dm_ = dm_;
  return model;
}*/

double LeoSandboxModel::export_meshup_animation(const Vector &action, const Vector &next, bool append) const
{
  std::ofstream data_stream;
  data_stream.open ("sd_leo.csv", append?(std::ios_base::app):(std::ios_base::trunc));
  if (!data_stream || 2*dof_count_ > next.size())
  {
    std::cerr << "Error opening file sd_leo.csv" << std::endl;
    abort();
  }
  data_stream << next[rlsTime] << ", ";
  for (int i = 0; i < 2*dof_count_-1; i++)
    data_stream << next[i] << ", ";
  data_stream << next[2*dof_count_-1] << std::endl;
  data_stream.close();

  data_stream.open ("u_leo.csv", append?(std::ios_base::app):(std::ios_base::trunc));
  if (!data_stream || dof_count_ > action.size())
  {
    std::cerr << "Error opening file u_leo.csv" << std::endl;
    abort();
  }
  data_stream << next[rlsTime] << ", ";
  for (int i = 0; i < dof_count_-1; i++)
    data_stream << action[i] << ", ";
  data_stream << action[dof_count_-1] << std::endl;
  data_stream.close();
}

///////////////////////////////////////////////

LeoSquattingSandboxModel *LeoSquattingSandboxModel::clone() const
{
  LeoSquattingSandboxModel *model = new LeoSquattingSandboxModel();
  model->dm_ = dm_;
  return model;
}

void LeoSquattingSandboxModel::start(const Vector &hint, Vector *state)
{
  if (target_env_)
  {
    // reduce state
    Vector state0;
    state0.resize(2*dof_count_);
    state0 << state->block(0, 0, 1, 2*dof_count_);

    target_env_->start(0, &state0);

    *state << state0,
              0.0,  // rlsTime
              0.0,//0.28, // rlsRefRootHeight, possible values 0.28 and 0.35
              ConstantVector(rlsStateDim - 2*rlsDofDim - 2, 0); // initialize the rest to zero
  }

  dm_.dynamics_->finalize(*state); // Fill parts of a state such as Center of Mass, Angular Momentum
  state_ = *state;

  TRACE("Initial state: " << state_);
}

double LeoSquattingSandboxModel::step(const Vector &action, Vector *next)
{
  // reduce state
  Vector state0;
  state0.resize(2*dof_count_+1);
  state0 << state_.block(0, 0, 1, 2*dof_count_+1);

  // auto-actuate arm
  Vector action0;
  action0.resize(dof_count_);
  if (action.size() == 3)
  {
    double armVoltage = (14.0/3.3) * 5.0*(-0.26 - state_[rlsArmAngle]);
    armVoltage = fmin(10.7, fmax(armVoltage, -10.7)); // ensure voltage within limits
    action0 << action, armVoltage;
  }
  else
    action0 << action;

  for (int i = 0; i < action_min_.size(); i++)
    action0[i] = fmin(action_max_[i], fmax(action0[i], action_min_[i])); // ensure voltage within limits

//  action0 << ConstantVector(4, 0);

//  std::cout << state0 << std::endl;
//  std::cout << "  > Action: " << action0 << std::endl;

  // call dynamics of the reduced state
  Vector next0;
  next0.resize(state0.size());
  double tau;
  if (target_env_)
  {
    tau = target_env_->step(action0, &next0, NULL, NULL);
    next0[rlsTime] = state0[rlsTime] + tau;
  }
  else
    tau = dm_.step(state0, action0, &next0);

  next->resize(2*dof_count_+2);
  *next << next0, state_[2*dof_count_+1]; // fake direction

  dm_.dynamics_->finalize(*next);

  if ( fabs((*next)[rlsComVelocityZ] - 0.0) < 0.01)
  {
    if ( fabs((*next)[rlsRootZ] - 0.28) < 0.01)
      (*next)[rlsRefRootZ] = 0.35;
    else if ( fabs((*next)[rlsRootZ] - 0.35) < 0.01)
      (*next)[rlsRefRootZ] = 0.28;
  }

  if ((*next)[rlsRefRootZ] != state_[rlsRefRootZ])
    (*next)[stsSquats] = state_[stsSquats] + 1;

//  std::cout << "  > Height: " << (*next)[rlsRootZ] << std::endl;
//  std::cout << "  > Next state: " << *next << std::endl;
//  export_meshup_animation(action0, *next, true);

  state_ = *next;
  return tau;
}
