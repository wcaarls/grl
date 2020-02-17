/** \file action.cpp
 * \brief Action and action-probability policies policies source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-16
 *
 * \copyright \verbatim
 * Copyright (c) 2015, Wouter Caarls
 * All rights reserved.
 *
 * This file is part of GRL, the Generic Reinforcement Learning library.
 *
 * GRL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * \endverbatim
 */

#include <grl/samplers/softmax.h>

#include <grl/policies/action.h>

using namespace grl;

REGISTER_CONFIGURABLE(ActionPolicy)
REGISTER_CONFIGURABLE(GaussianPolicy)
REGISTER_CONFIGURABLE(StochasticPolicy)

// *** ActionPolicy

void ActionPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("sigma", "Standard deviation of Gaussian exploration distribution", sigma_, CRP::Configuration));
  config->push_back(CRP("theta", "Ornstein-Uhlenbeck friction term (1=pure Gaussian noise)", theta_, CRP::Configuration));

  config->push_back(CRP("decay_rate", "Multiplicative decay factor per episode", decay_rate_, CRP::Configuration));
  config->push_back(CRP("decay_min", "Minimum decay (sigma_min = sigma*decay_min)", decay_min_, CRP::Configuration));
  
  config->push_back(CRP("renormalize", "int", "Renormalize representation output from [-1, 1] to [min, max]", renormalize_, CRP::Configuration, 0, 1));

  config->push_back(CRP("output_min", "vector.action_min", "Lower limit on outputs", min_, CRP::System));
  config->push_back(CRP("output_max", "vector.action_max", "Upper limit on outputs", max_, CRP::System));

  config->push_back(CRP("projector", "projector.observation", "Projects observations onto representation space", projector_));
  config->push_back(CRP("representation", "representation.action", "Action representation", representation_));
}

void ActionPolicy::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  
  renormalize_ = config["renormalize"];
  sigma_ = config["sigma"].v();
  theta_ = config["theta"].v();
  decay_rate_ = config["decay_rate"];
  decay_min_ = config["decay_min"];
  decay_ = 1.;
  min_ = config["output_min"].v();
  max_ = config["output_max"].v();
  
  if (min_.size() != max_.size() || !min_.size())
    throw bad_param("policy/action:{output_min,output_max}");
  
  if (!sigma_.size())
    sigma_ = VectorConstructor(0.);
    
  if (sigma_.size() == 1)
    sigma_ = ConstantVector(min_.size(), sigma_[0]);
    
  if (sigma_.size() != min_.size())
    throw bad_param("policy/action:sigma");
    
  if (!theta_.size())
    theta_ = VectorConstructor(1.);
    
  if (theta_.size() == 1)
    theta_ = ConstantVector(min_.size(), theta_[0]);
    
  if (theta_.size() != min_.size())
    throw bad_param("policy/action:theta");
}

void ActionPolicy::reconfigure(const Configuration &config)
{
  if (config.has("action") && config["action"].str() == "reset")
    decay_ = 1;
}

double ActionPolicy::read(const Vector &in, Vector *result) const
{
  ProjectionPtr p = projector_->project(in);
  representation_->read(p, result);
  
  // Some representations may not always return a value.
  if (!result->size())
    *result = (min_+max_)/2;
  else if (renormalize_)
    for (size_t ii=0; ii < result->size(); ++ii)
      (*result)[ii] = (*result)[ii] * (max_[ii]-min_[ii])/2 + (min_[ii]+max_[ii])/2;
    
  return (*result)[0];
}

void ActionPolicy::read(const Matrix &in, Matrix *result) const
{
  representation_->batchRead(in.rows());
  for (size_t ii=0; ii != in.rows(); ++ii)
    representation_->enqueue(projector_->project(in.row(ii)));
  representation_->read(result);
  
  if (renormalize_)
    for (size_t rr=0; rr < result->rows(); ++rr)
      for (size_t cc=0; cc < result->cols(); ++cc)
        (*result)(rr, cc) = (*result)(rr, cc) * (max_[cc]-min_[cc])/2 + (min_[cc]+max_[cc])/2;
}

void ActionPolicy::act(double time, const Observation &in, Action *out)
{
  read(in, &out->v);
  out->type = atGreedy;
  
  if (sigma_.size() != out->size())
  {
    ERROR("Expected action size " << sigma_.size() << ", representation produced " << out->size());
    throw bad_param("policy/action:{output_min,output_max}");
  }

  if (time == 0 || n_.size() != out->size())
    n_ = ConstantVector(out->size(), 0.);

  if (time == 0.)
    decay_ = fmax(decay_*decay_rate_, decay_min_);
    
  out->logp = 0;

  for (size_t ii=0; ii < out->size(); ++ii)
  {
    if (sigma_[ii])
    {
      n_[ii] = (1-theta_[ii])*n_[ii] + RandGen::getNormal(0., decay_*sigma_[ii]);
      (*out)[ii] += n_[ii];
      out->type = atExploratory;
      out->logp += lognormal(n_[ii], decay_*sigma_[ii]);
    }

    (*out)[ii] = fmin(fmax((*out)[ii], min_[ii]), max_[ii]);
  }
}

void ActionPolicy::act(const Observation &in, Action *out) const
{
  read(in, &out->v);
  out->type = atGreedy;
  
  if (sigma_.size() != out->size())
  {
    ERROR("Expected action size " << sigma_.size() << ", representation produced " << out->size());
    throw bad_param("policy/action:{output_min,output_max}");
  }
  
  out->logp = 0;

  for (size_t ii=0; ii < out->size(); ++ii)
  {
    if (sigma_[ii])
    {
      double r = RandGen::getNormal(0., decay_*sigma_[ii]);
      (*out)[ii] += r;
      out->type = atExploratory;
      out->logp += lognormal(r, decay_*sigma_[ii]);
    }
      
    (*out)[ii] = fmin(fmax((*out)[ii], min_[ii]), max_[ii]);
  }
}

// *** GaussianPolicy

void GaussianPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("renormalize", "int", "Renormalize representation output from [-1, 1] to [min, max]", renormalize_, CRP::Configuration, 0, 1));

  config->push_back(CRP("output_min", "vector.action_min", "Lower limit on outputs", min_, CRP::System));
  config->push_back(CRP("output_max", "vector.action_max", "Upper limit on outputs", max_, CRP::System));

  config->push_back(CRP("projector", "projector.observation", "Projects observations onto representation space", projector_));
  config->push_back(CRP("representation", "representation.action/gaussian", "Action-logstd representation", representation_));
}

void GaussianPolicy::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  
  renormalize_ = config["renormalize"];
  min_ = config["output_min"].v();
  max_ = config["output_max"].v();
  
  if (min_.size() != max_.size() || !min_.size())
    throw bad_param("policy/action:{output_min,output_max}");
    
  dims_ = min_.size();
}

void GaussianPolicy::reconfigure(const Configuration &config)
{
}

double GaussianPolicy::read(const Vector &in, Vector *result) const
{
  ProjectionPtr p = projector_->project(in);
  representation_->read(p, result);
  
  // Some representations may not always return a value.
  if (!result->size())
  {
    *result = (min_+max_)/2;
    *result = extend(*result, (Vector)Vector::Zero(dims_));
    return (*result)[0];
  }
  else if (renormalize_)
  {
    if (result->size() < 2*dims_)
    {
      ERROR("Expected action size 2*" << dims_ << ", representation produced " << result->size());
      throw bad_param("mapping/policy/action/stochastic:{output_min,output_max}");
    }
  
    for (size_t ii=0; ii < dims_; ++ii)
    {
      (*result)[      ii] = (*result)[      ii] * (max_[ii]-min_[ii])/2 + (min_[ii]+max_[ii])/2;
      (*result)[dims_+ii] = (*result)[dims_+ii] + std::log((max_[ii]-min_[ii])/2);
    }
  }
    
  return (*result)[0];
}

void GaussianPolicy::read(const Matrix &in, Matrix *result) const
{
  representation_->batchRead(in.rows());
  for (size_t ii=0; ii != in.rows(); ++ii)
    representation_->enqueue(projector_->project(in.row(ii)));
  representation_->read(result);

  if (result->cols() < 2*dims_)
  {
    ERROR("Expected action size 2*" << dims_ << ", representation produced " << result->cols());
    throw bad_param("mapping/policy/action/stochastic:{output_min,output_max}");
  }
  
  if (renormalize_)
    for (size_t rr=0; rr < result->rows(); ++rr)
      for (size_t cc=0; cc < dims_; ++cc)
      {
        (*result)(rr,       cc) = (*result)(rr,       cc) * (max_[cc]-min_[cc])/2 + (min_[cc]+max_[cc])/2;
        (*result)(rr, dims_+cc) = (*result)(rr, dims_+cc) + std::log((max_[cc]-min_[cc])/2);
      }
}

void GaussianPolicy::act(const Observation &in, Action *out) const
{
  Vector result;
  read(in, &result);
  
  if (result.size() < 2*dims_)
  {
    ERROR("Expected action size 2*" << dims_ << ", representation produced " << result.size());
    throw bad_param("mapping/policy/action/gaussian:{output_min,output_max}");
  }

  *out = result.head(dims_);
  out->type = atExploratory;
  out->logp = 0;

  for (size_t ii=0; ii < out->size(); ++ii)
  {
    double sigma = exp(result[dims_+ii]);
    double r = RandGen::getNormal(0., sigma);
    double mu = (*out)[ii];

    (*out)[ii] += r;
    (*out)[ii] = fmin(fmax((*out)[ii], min_[ii]), max_[ii]);

    // TODO: Proper clipping using integral over pdf outside range.
    out->logp += lognormal((*out)[ii]-mu, sigma);
  }
}

// *** StochasticPolicy

void StochasticPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("renormalize", "int", "Renormalize representation output from [-1, 1] to [min, max]", renormalize_, CRP::Configuration, 0, 1));

  config->push_back(CRP("output_min", "vector.action_min", "Lower limit on outputs", min_, CRP::System));
  config->push_back(CRP("output_max", "vector.action_max", "Upper limit on outputs", max_, CRP::System));

  config->push_back(CRP("projector", "projector.observation", "Projects observations onto representation space", projector_));
  config->push_back(CRP("representation", "representation.action/stochastic", "Action-logp representation", representation_));
}

void StochasticPolicy::configure(Configuration &config)
{
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  
  renormalize_ = config["renormalize"];
  min_ = config["output_min"].v();
  max_ = config["output_max"].v();
  
  if (min_.size() != max_.size() || !min_.size())
    throw bad_param("policy/action:{output_min,output_max}");
    
  dims_ = min_.size();
}

void StochasticPolicy::reconfigure(const Configuration &config)
{
}

double StochasticPolicy::read(const Vector &in, Vector *result) const
{
  ProjectionPtr p = projector_->project(in);
  representation_->read(p, result);
  
  // Some representations may not always return a value.
  if (!result->size())
  {
    *result = (min_+max_)/2;
    *result = extend(*result, (Vector)Vector::Zero(1));
    return (*result)[0];
  }
  else if (renormalize_)
  {
    if (result->size() < dims_+1)
    {
      ERROR("Expected action size " << dims_ << "+1, representation produced " << result->size());
      throw bad_param("mapping/policy/action/stochastic:{output_min,output_max}");
    }
  
    for (size_t ii=0; ii < dims_; ++ii)
      (*result)[ii] = (*result)[ii] * (max_[ii]-min_[ii])/2 + (min_[ii]+max_[ii])/2;
  }
    
  return (*result)[0];
}

void StochasticPolicy::read(const Matrix &in, Matrix *result) const
{
  representation_->batchRead(in.rows());
  for (size_t ii=0; ii != in.rows(); ++ii)
    representation_->enqueue(projector_->project(in.row(ii)));
  representation_->read(result);

  if (result->cols() < dims_+1)
  {
    ERROR("Expected action size " << dims_ << "+1, representation produced " << result->size());
    throw bad_param("mapping/policy/action/stochastic:{output_min,output_max}");
  }
  
  if (renormalize_)
    for (size_t rr=0; rr < result->rows(); ++rr)
      for (size_t cc=0; cc < dims_; ++cc)
        (*result)(rr, cc) = (*result)(rr, cc) * (max_[cc]-min_[cc])/2 + (min_[cc]+max_[cc])/2;
}

void StochasticPolicy::act(const Observation &in, Action *out) const
{
  Vector result;
  read(in, &result);
  
  if (result.size() < dims_+1)
  {
    ERROR("Expected action size " << dims_ << "+1, representation produced " << result.size());
    throw bad_param("mapping/policy/action/stochastic:{output_min,output_max}");
  }

  *out = result.head(dims_);

  for (size_t ii=0; ii < dims_; ++ii)
    (*out)[ii] = fmin(fmax((*out)[ii], min_[ii]), max_[ii]);
  
  out->logp = result(dims_);
  if (out->logp == 0)
    out->type = atGreedy;
  else
    out->type = atExploratory;
}
