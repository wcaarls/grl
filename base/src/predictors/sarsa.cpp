#include <grl/predictors/sarsa.h>

using namespace grl;

REGISTER_CONFIGURABLE(SARSAPredictor)
REGISTER_CONFIGURABLE(ExpectedSARSAPredictor)

void SARSAPredictor::request(ConfigurationRequest *config)
{
}

void SARSAPredictor::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  trace_ = (Trace*)config["trace"].ptr();
  
  alpha_ = config["alpha"];
  gamma_ = config["gamma"];
  lambda_ = config["lambda"];
}

void SARSAPredictor::reconfigure(const Configuration &config)
{
}

SARSAPredictor *SARSAPredictor::clone() const
{
  SARSAPredictor *sp = new SARSAPredictor();
  sp->projector_ = projector_->clone();
  sp->representation_= representation_->clone();
  sp->trace_ = trace_->clone();
  return sp;
}

void SARSAPredictor::update(const Transition &transition)
{
  ProjectionPtr p = projector_->project(transition.prev_obs, transition.prev_action),
                pp = projector_->project(transition.obs, transition.action);

  Vector q, qp;
  representation_->read(p, &q);
  representation_->read(pp, &qp);

  double delta = transition.reward + gamma_*qp[0] - q[0];

  trace_->add(p, gamma_*lambda_);
  representation_->update(*trace_, VectorConstructor(alpha_*delta));
}

void SARSAPredictor::finalize()
{
  trace_->clear();
}

void ExpectedSARSAPredictor::request(ConfigurationRequest *config)
{
}

void ExpectedSARSAPredictor::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  policy_ = (QPolicy*)config["policy"].ptr();
  sampler_ = (Sampler*)config["sampler"].ptr();
  trace_ = (Trace*)config["trace"].ptr();
  
  alpha_ = config["alpha"];
  gamma_ = config["gamma"];
  lambda_ = config["lambda"];
}

void ExpectedSARSAPredictor::reconfigure(const Configuration &config)
{
}

ExpectedSARSAPredictor *ExpectedSARSAPredictor::clone() const
{
  ExpectedSARSAPredictor *sp = new ExpectedSARSAPredictor();
  sp->projector_ = projector_->clone();
  sp->representation_= representation_->clone();
  sp->policy_ = policy_->clone();
  sp->sampler_ = sampler_->clone();
  sp->trace_ = trace_->clone();
  return sp;
}

void ExpectedSARSAPredictor::update(const Transition &transition)
{
  ProjectionPtr p = projector_->project(transition.prev_obs, transition.prev_action);

  Vector values, distribution;
  policy_->values(transition.obs, &values);
  sampler_->distribution(values, &distribution);

  double v = 0.;
  for (size_t ii=0; ii < values.size(); ++ii)
    v += values[ii]*distribution[ii];

  Vector q;
  representation_->read(p, &q);

  double delta = transition.reward + gamma_*v - q[0];

  trace_->add(p, gamma_*lambda_);
  representation_->update(*trace_, VectorConstructor(alpha_*delta));
}

void ExpectedSARSAPredictor::finalize()
{
  trace_->clear();
}


