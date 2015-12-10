/** \file state_feedback.cpp
 * \brief State feedback policy source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-27
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

#include <grl/policies/state_feedback.h>

#define P(i, o) ((i)*outputs_+(o))

using namespace grl;

REGISTER_CONFIGURABLE(StateFeedbackPolicy)
REGISTER_CONFIGURABLE(SampleFeedbackPolicy)

// StateFeedbackPolicy

void StateFeedbackPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("operating_state", "Operating state around which gains are defined", operating_state_));
  config->push_back(CRP("operating_action", "Operating action around which gains are defined", operating_action_));
  
  config->push_back(CRP("gains", "Gains ([in1_out1, ..., in1_outN, ..., inN_out1, ..., inN_outN])", gains_, CRP::Online));
  
  config->push_back(CRP("output_min", "vector.action_min", "Lower action limit", min_, CRP::System));
  config->push_back(CRP("output_max", "vector.action_max", "Upper action limit", max_, CRP::System));
}

void StateFeedbackPolicy::configure(Configuration &config)
{
  operating_state_ = config["operating_state"];
  if (!operating_state_.size())
    throw bad_param("policy/parameterized/state_feedback:operating_state");

  operating_action_ = config["operating_action"];
  if (!operating_action_.size())
    throw bad_param("policy/parameterized/state_feedback:operating_action");
    
  gains_ = config["gains"];
  if (!gains_.size())
    gains_ = ConstantVector(operating_state_.size()*operating_action_.size(), 0.);
  if (gains_.size() != operating_state_.size()*operating_action_.size())
    throw bad_param("policy/parameterized/state_feedback:{gains,operating_state,operating_action}");
    
  min_ = config["output_min"];
  max_ = config["output_max"];
  
  if (min_.size() != operating_action_.size())
    throw bad_param("policy/parameterized/state_feedback:min");
    
  if (max_.size() != operating_action_.size())
    throw bad_param("policy/parameterized/state_feedback:max");
}

void StateFeedbackPolicy::reconfigure(const Configuration &config)
{
}

StateFeedbackPolicy *StateFeedbackPolicy::clone() const
{
  return new StateFeedbackPolicy(*this);
}

void StateFeedbackPolicy::act(const Vector &in, Vector *out) const
{
  if (in.size() != operating_state_.size())
    throw bad_param("policy/parameterized/state_feedback:operating_state");

  out->resize(operating_action_.size()); 
  
  // If all gains are zero (== undefined), apply random action
  if (gains_.minCoeff() == 0 && gains_.maxCoeff() == 0)
  {
    *out = min_ + RandGen::getVector(min_.size())*(max_-min_);
    return;
  }

  for (size_t oo=0; oo < out->size(); ++oo)
  {
    double u = 0;
    
    for (size_t ii=0; ii < operating_state_.size(); ++ii)
    {
      double x = operating_state_[ii] - in[ii];
      
      u += gains_[ii*out->size()+oo]*x;
    }
    
    (*out)[oo] = fmin(fmax(operating_action_[oo]+u, min_[oo]), max_[oo]);
  }
}

// SampleFeedbackPolicy

void SampleFeedbackPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("output_min", "vector.action_min", "Lower action limit", min_, CRP::System));
  config->push_back(CRP("output_max", "vector.action_max", "Upper action limit", max_, CRP::System));
}

void SampleFeedbackPolicy::configure(Configuration &config)
{
  min_ = config["output_min"];
  max_ = config["output_max"];
  
  if (min_.size() != max_.size())
    throw bad_param("policy/sample_feedback:{min,max}");
}

void SampleFeedbackPolicy::reconfigure(const Configuration &config)
{
}

SampleFeedbackPolicy *SampleFeedbackPolicy::clone() const
{
  return new SampleFeedbackPolicy(*this);
}

void SampleFeedbackPolicy::act(const Vector &in, Vector *out) const
{
  if (!samples_.size())
  {
    WARNING("Applying default action");
    *out = (max_-min_)/2;
    return;
  }

  if (in.size() != samples_.front().x.size())
  {
    ERROR("Received input size " << in.size() << ", but samples have input size " << samples_.front().x.size());
    throw Exception("Invalid input size");
  }

  // Find closest sample
  double dist_min = std::numeric_limits<double>::infinity();
  const Sample *sample = NULL;
  for (auto ii=samples_.begin(); ii !=samples_.end(); ++ii)
  {
    double dist = 0;
    for (size_t jj=0; jj < in.size(); ++jj)
      dist += fabs(in[jj]-ii->x[jj]);
    if (dist < dist_min)
    {
      dist_min = dist;
      sample = &*ii;
    }
  }      
  
    
  // Apply local controller
  *out = sample->u;
  Vector control(out->size());

  for (size_t oo=0; oo < out->size(); ++oo)
  {
    double u = 0;
    
    for (size_t ii=0; ii < in.size(); ++ii)
    {
      double x = sample->x[ii] - in[ii];
      
      u += sample->L(oo,ii)*x;
    }

    control[oo] = u;
  }

  CRAWL("Closest sample " << sample->x << " -> " << sample->u << " + " << control << " feedback");

  for (size_t oo=0; oo < out->size(); ++oo)
    (*out)[oo] = fmin(fmax((*out)[oo]+control[oo], min_[oo]), max_[oo]);
}

void SampleFeedbackPolicy::clear()
{
  samples_.clear();
}

void SampleFeedbackPolicy::push(const Sample &sample)
{
  samples_.push_back(sample);
}
        