#include <grl/agents/dyna.h>

using namespace grl;

REGISTER_CONFIGURABLE(DynaAgent)

void DynaAgent::request(ConfigurationRequest *config)
{
}

void DynaAgent::configure(Configuration &config)
{
  policy_ = (Policy*)config["policy"].ptr();
  predictor_ = (Predictor*)config["predictor"].ptr();

  planning_steps_ = config["planning_steps"];
  
  model_agent_ = (Agent*)config["model_agent"].ptr();
  model_projector_ = (Projector*)config["model_projector"].ptr();
  model_representation_ = (Representation*)config["model_representation"].ptr();
  
  wrapping_ = config["wrapping"];
}

void DynaAgent::reconfigure(const Configuration &config)
{
}

DynaAgent *DynaAgent::clone() const
{
  DynaAgent *agent = new DynaAgent(*this);
  
  agent->policy_ = policy_->clone();
  agent->predictor_= predictor_->clone();
  agent->model_agent_= model_agent_->clone();
  agent->model_projector_= model_projector_->clone();
  agent->model_representation_= model_representation_->clone();
  
  return agent;
}

void DynaAgent::start(const Vector &obs, Vector *action)
{
  predictor_->finalize();
  policy_->act(obs, action);
  
  prev_obs_ = obs;
  prev_action_ = *action;
}

void DynaAgent::step(const Vector &obs, double reward, Vector *action)
{
  policy_->act(obs, action);
  
  Transition t(prev_obs_, prev_action_, reward, obs, *action);
  predictor_->update(t);
  
  learnModel(t);
  runModel();

  prev_obs_ = obs;
  prev_action_ = *action;  
}

void DynaAgent::end(double reward)
{
  Transition t(prev_obs_, prev_action_, reward);
  predictor_->update(t);
  
  t.obs = prev_obs_;
  t.action = prev_action_;
  learnModel(t);
  runModel();
}

void DynaAgent::learnModel(const Transition &t)
{
  ProjectionPtr p = model_projector_->project(extend(t.prev_obs, t.prev_action));
  Vector target;
  
  if (!t.obs.empty())
  {
    target = t.obs-t.prev_obs;
    
    for (size_t ii=0; ii < target.size(); ++ii)
      if (wrapping_[ii])
      {
        if (target[ii] > 0.5*wrapping_[ii])
          target[ii] -= wrapping_[ii];
        else if (target[ii] < -0.5*wrapping_[ii])
          target[ii] += wrapping_[ii];
      }
    
    target.push_back(t.reward);
    target.push_back(0);
  }
  else
  {
    // Absorbing state
    target.resize(t.prev_obs.size(), 0.);
    target.push_back(t.reward);
    target.push_back(1);
  }
  
  model_representation_->write(p, target);
}

void DynaAgent::stepModel(const Vector &obs, const Vector &action, Vector *next, double *reward, int *terminal)
{
  ProjectionPtr p = model_projector_->project(extend(obs, action)); 
  
  if (!p)
  {
    next->clear();
    return;
  }
 
  model_representation_->read(p, next);
  
  for (size_t ii=0; ii < next->size(); ++ii)
    (*next)[ii] += obs[ii];
    
  // Guard against failed model prediction
  if (!next->empty())
  {
    *reward = (*next)[next->size()-2];  
    *terminal = (*next)[next->size()-1];
    next->resize(next->size()-2);
  }
}

void DynaAgent::runModel()
{
  Vector obs = prev_obs_, action;
  int terminal=1;

  for (size_t ii=0; ii < planning_steps_; ++ii)
  {
    if (terminal)
      model_agent_->start(obs, &action);
      
    Vector next;
    double reward;
  
    stepModel(obs, action, &next, &reward, &terminal);

    // Guard against failed model prediction    
    if (!next.empty())
    {
      if (terminal)
        model_agent_->end(reward);
      else
        model_agent_->step(obs, reward, &action);
    }
    else
      terminal = 1;
  }
}
