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

QPolicy *QPolicy::clone() const
{
  QPolicy *qp = new QPolicy();
  qp->discretizer_ = discretizer_->clone();
  qp->projector_ = projector_->clone();
  qp->representation_ = representation_->clone();
  qp->sampler_ = sampler_->clone();
  
  return qp;
}

double QPolicy::value(const Vector &in) const
{
  LargeVector qvalues, distribution;
  double v=0;
  
  values(in, &qvalues);
  
  sampler_->distribution(qvalues, &distribution);
  
  for (size_t ii=0; ii < qvalues.size(); ++ii)
    v += qvalues[ii]*distribution[ii];
    
  return v;
}

/**
 * @brief QPolicy::values calculates Q(s,a) values for provided state 'in' and for all actions 'a'
 * @param in: state
 * @param out: each element containes an (e.g. LLR) approximation of a value function Q(s, a), whrere 'a' is an index of vector 'out'
 */
void QPolicy::values(const Vector &in, LargeVector *out) const
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

TransitionType QPolicy::act(const Vector &in, Vector *out) const
{
  LargeVector qvalues;
  TransitionType tt;
  
  values(in, &qvalues);
  size_t action = sampler_->sample(qvalues, tt);
  
  *out = discretizer_->at(in, action);
  TRACE(*out);

  return tt;
}

//Vector prev_out;
TransitionType QPolicy::act(double time, const Vector &in, Vector *out)
{
  LargeVector qvalues;
  TransitionType tt;

  values(in, &qvalues);
  size_t action = sampler_->sample(time, qvalues, tt);

  *out = discretizer_->at(in, action);
  TRACE(*out);
/*
  // Verifying neighbouring action in case of PADA
  if (prev_out.size())
  {
    for (int i = 0; i < prev_out.size(); i++)
      if (fabs((*out)[i] - prev_out[i]) > 5.35)
        std::cout << "Something is wrong!" << std::endl;
  }
  prev_out = *out;
*/
  return tt;
}
