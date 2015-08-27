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

void StateFeedbackPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("operating_state", "Operating state around which gains are defined", operating_state_));
  config->push_back(CRP("operating_action", "Operating action around which gains are defined", operating_action_));
  
  config->push_back(CRP("gains", "Gains ([in1_out1, ..., in1_outN, ..., inN_out1, ..., inN_outN])", gains_, CRP::Online));
}

void StateFeedbackPolicy::configure(Configuration &config)
{
  operating_state_ = config["operating_state"];
  if (operating_state_.empty())
    throw bad_param("policy/parameterized/state_feedback:operating_state");

  operating_action_ = config["operating_action"];
  if (operating_action_.empty())
    throw bad_param("policy/parameterized/state_feedback:operating_action");
    
  gains_ = config["gains"];
  if (gains_.empty())
    gains_.resize(operating_state_.size()*operating_action_.size(), 0.);
  if (gains_.size() != operating_state_.size()*operating_action_.size())
    throw bad_param("policy/parameterized/state_feedback:{gains,operating_state,operating_action}");
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

  for (size_t oo=0; oo < out->size(); ++oo)
  {
    double u = 0;
    
    for (size_t ii=0; ii < operating_state_.size(); ++ii)
    {
      double x = operating_state_[ii] - in[ii];
      
      u += gains_[ii*out->size()+oo]*x;
    }
    
    (*out)[oo] = operating_action_[oo]+u;
  }
}
