/** \file q.cpp
 * \brief Q policy source file. 
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-01-22
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

#include <grl/policies/q.h>

using namespace grl;

REGISTER_CONFIGURABLE(QPolicy)
REGISTER_CONFIGURABLE(QVectorPolicy)

void QPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("projector", "projector.pair", "Projects observation-action pairs onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/action", "Action-value representation", representation_));
  config->push_back(CRP("sampler", "sampler", "Samples actions from action-values", sampler_));
}

void QPolicy::configure(Configuration &config)
{
  discretizer_ = (Discretizer*)config["discretizer"].ptr();
  
  projector_ = (Projector*)config["projector"].ptr();
  representation_ = (Representation*)config["representation"].ptr();
  sampler_ = (Sampler*)config["sampler"].ptr();
}

void QPolicy::reconfigure(const Configuration &config)
{
}

double QPolicy::value(const Observation &in) const
{
  LargeVector qvalues, dist;
  double v=0;
  
  values(in, &qvalues);
  sampler_->distribution(qvalues, &dist);
  
  for (size_t ii=0; ii < dist.size(); ++ii)
    v += qvalues[ii]*dist[ii];
    
  return v;
}

void QPolicy::value(std::vector<const Observation*> &in, std::vector<const Action*> &prev, LargeVector *out) const
{
  Matrix qvalues;
  LargeVector dist;

  values(in, &qvalues);
  out->resize(in.size());
  
  for (size_t ii=0; ii < in.size(); ++ii)
  {
    double v = 0;
    
    sampler_->distribution(qvalues.row(ii), &dist);
    for (size_t jj=0; jj < dist.size(); ++jj)
      v += qvalues(ii,jj)*dist[jj];
    (*out)[ii] = v;
  }
}

/**
 * @brief QPolicy::values calculates Q(s,a) values for provided state 'in' and for all actions 'a'
 * @param in: state
 * @param out: each element containes an (e.g. LLR) approximation of a value function Q(s, a), whrere 'a' is an index of vector 'out'
 */
void QPolicy::values(const Observation &in, LargeVector *out) const
{
  // 'projections' contains list of neighbours around state 'in' and any possible action. Number of projections is equal to number of possible actions.
  std::vector<Vector> variants;
  std::vector<ProjectionPtr> projections;

  discretizer_->options(in, &variants);
  projector_->project(in, variants, &projections);

  out->resize(variants.size());
  Vector value;
  for (size_t ii=0; ii < variants.size(); ++ii)
    (*out)[ii] = representation_->read(projections[ii], &value); // reading approximated values
}

void QPolicy::values(std::vector<const Observation*> &in, Matrix *out) const
{
  std::vector<Vector> variants;
  std::vector<ProjectionPtr> projections;

  representation_->batchRead(in.size()*variants.size());
  for (size_t ii=0; ii != in.size(); ++ii)
  {
    discretizer_->options(*in[ii], &variants);
    projector_->project(*in[ii], variants, &projections);
    for (size_t jj=0; jj < variants.size(); ++jj)
      representation_->enqueue(projections[jj]);
  }
  
  representation_->read(out);
  
  *out = out->col(0);
  out->resize(in.size(), variants.size());
}

void QPolicy::act(const Observation &in, Action *out) const
{
  LargeVector qvalues;
  ActionType at;
  double logp;
  
  values(in, &qvalues);
  size_t action = sampler_->sample(qvalues, &at, &logp);
  
  *out = discretizer_->at(in, action);
  out->type = at;
  out->logp = logp;
}

void QPolicy::act(double time, const Observation &in, Action *out)
{
  LargeVector qvalues;
  ActionType at;
  double logp;

  values(in, &qvalues);
  size_t action = sampler_->sample(time, qvalues, &at, &logp);

  *out = discretizer_->at(in, action);
  out->type = at;
  out->logp = logp;
}

void QPolicy::distribution(const Observation &in, const Action &prev, LargeVector *out) const
{
  LargeVector qvalues;

  values(in, &qvalues);
  sampler_->distribution(qvalues, out);
}

void QPolicy::distribution(std::vector<const Observation*> &in, std::vector<const Action*> &prev, Matrix *out) const
{
  Matrix qvalues;
  LargeVector row;
  
  values(in, &qvalues);
  out->resize(qvalues.rows(), qvalues.cols());
  for (size_t ii=0; ii != in.size(); ++ii)
  {
    sampler_->distribution(qvalues.row(ii), &row);
    out->row(ii) = row;
  }
}

void QVectorPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("discretizer", "discretizer.action", "Action discretizer", discretizer_));
  config->push_back(CRP("projector", "projector.state", "Projects observations onto representation space", projector_));
  config->push_back(CRP("representation", "representation.value/actions", "Action-value vector representation", representation_));
  config->push_back(CRP("sampler", "sampler", "Samples actions from action-values", sampler_));
}

void QVectorPolicy::values(const Observation &in, LargeVector *out) const
{
#ifdef GRL_VECTOR_IS_LARGE_VECTOR
  Vector tmp;

  representation_->read(projector_->project(in), &tmp);
  
  *out = tmp;
#else
  Matrix qvalues;

  representation_->batchRead(1);
  representation_->enqueue(projector_->project(in));
  representation_->read(&qvalues);
  *out = qvalues.row(0);
#endif
}

void QVectorPolicy::values(std::vector<const Observation*> &in, Matrix *out) const
{
  representation_->batchRead(in.size());
  for (size_t ii=0; ii != in.size(); ++ii)
    representation_->enqueue(projector_->project(*in[ii]));
  representation_->read(out);
}
