/** \file dsp.cpp
 * \brief DSP agent source file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2022-05-27
 *
 * \copyright \verbatim
 * Copyright (c) 2022, Wouter Caarls
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
 
#include <grl/agents/dsp.h>

using namespace grl;

REGISTER_CONFIGURABLE(DSPAgent)

void DSPAgent::request(ConfigurationRequest *config)
{
  config->push_back(CRP("wrapping", "vector.wrapping", "Input wrapping boundaries", wrapping_));

  config->push_back(CRP("input_filters", "filter", "Filters applied to agent input", &input_filters_, true));
  config->push_back(CRP("output_filters", "filter", "Filters applied to agent output", &output_filters_, true));
  config->push_back(CRP("agent", "agent", "Downstream agent", agent_));

  config->push_back(CRP("state", "signal/vector.state", "Current filtered state", CRP::Provided));
  config->push_back(CRP("action", "signal/vector.action", "Current filtered action", CRP::Provided));
}

void DSPAgent::configure(Configuration &config)
{
  wrapping_ = config["wrapping"].v();
  if (config["input_filters"].ptr())
    input_filters_ = *(ConfigurableList*)config["input_filters"].ptr();
  if (config["output_filters"].ptr())
    output_filters_ = *(ConfigurableList*)config["output_filters"].ptr();
  agent_ = (Agent*)config["agent"].ptr();
  
  state_ = new VectorSignal();
  action_ = new VectorSignal();
  
  config.set("state", state_);
  config.set("action", action_);
}

void DSPAgent::reconfigure(const Configuration &config)
{
}

void DSPAgent::start(const Observation &obs, Action *action)
{
  Observation o = obs;
  o.v = apply(input_filters_, obs.v, true, true);
  
  agent_->start(o, action);
  Vector ua = action->v;
  action->v = apply(output_filters_, action->v, false, true);

  state_->set(extend(obs.v, o.v));
  action_->set(extend(ua, action->v));
}

void DSPAgent::step(double tau, const Observation &obs, double reward, Action *action)
{
  Observation o = obs;
  o.v = apply(input_filters_, obs.v, true);

  agent_->step(tau, o, reward, action);
  Vector ua = action->v;
  action->v = apply(output_filters_, action->v, false);

  state_->set(extend(obs.v, o.v));
  action_->set(extend(ua, action->v));
}

void DSPAgent::end(double tau, const Observation &obs, double reward)
{
  Observation o = obs;
  o.v = apply(input_filters_, obs.v, true);

  agent_->end(tau, o, reward);  
}

Vector DSPAgent::apply(TypedConfigurableList<Filter> &filters, Vector sample, bool _wrap, bool reset)
{
  if (_wrap && wrapping_.size())
  {
    if (wrapping_.size() != sample.size())
      throw bad_param("agent/dsp/wrapping");
      
    if (!reset)
      for (size_t ii=0; ii != sample.size(); ++ii)
        if (wrapping_[ii])
        {
          double pm = wrap(prev_obs_[ii], wrapping_[ii]), pd = prev_obs_[ii] - pm;
          
          if (fabs(pm - sample[ii]) > wrapping_[ii]/2)
          {
            if (pm > sample[ii])
              sample[ii] += wrapping_[ii];
            else
              sample[ii] -= wrapping_[ii];
          }
          sample[ii] += pd;
        }
  
    prev_obs_ = sample;
  }

  for (size_t ii=0; ii != filters.size(); ++ii)
  {
    if (reset)
      filters[ii]->clear();
      
    sample = filters[ii]->filter(sample);
  }
  
  if (_wrap && wrapping_.size())
  {
    for (size_t ii=0; ii != sample.size(); ++ii)
      if (wrapping_[ii])
        sample[ii] = wrap(sample[ii], wrapping_[ii]);
  }
  
  return sample;
}
