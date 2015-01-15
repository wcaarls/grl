#include <grl/agents/td.h>

using namespace grl;

REGISTER_CONFIGURABLE(TDAgent)

void TDAgent::request(ConfigurationRequest *config)
{
}

void TDAgent::configure(const Configuration &config)
{
  policy_ = (Policy*)config["policy"].ptr();
  predictor_ = (Predictor*)config["predictor"].ptr();
}

void TDAgent::reconfigure(const Configuration &config)
{
}

TDAgent *TDAgent::clone() const
{
  TDAgent *agent = new TDAgent();
  agent->policy_ = policy_->clone();
  agent->predictor_= predictor_->clone();
  
  return agent;
}

void TDAgent::start(const Vector &obs, Vector *action)
{
  predictor_->finalize();
  policy_->act(obs, action);
  
  prev_obs_ = obs;
  prev_action_ = *action;
}

void TDAgent::step(const Vector &obs, double reward, Vector *action)
{
  policy_->act(obs, action);
  predictor_->update(Transition(prev_obs_, prev_action_, reward, obs, *action));

  prev_obs_ = obs;
  prev_action_ = *action;  
}

void TDAgent::end(double reward)
{
  predictor_->update(Transition(prev_obs_, prev_action_, reward));
}
