/** \file ddpg.h
 * \brief DDPG predictor header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2018-04-12
 *
 * \copyright \verbatim
 * Copyright (c) 2018, Wouter Caarls
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

#ifndef GRL_NAF_PREDICTOR_H_
#define GRL_NAF_PREDICTOR_H_

#include <grl/configurable.h>
#include <grl/predictor.h>
#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/trace.h>
#include <grl/policy.h>
#include <grl/policies/q.h>
#include <grl/mapping.h>

#include <grl/representations/tensorflow.h>

namespace grl
{

/// Deep Deterministic Policy Gradient Actor-Critic predictor.
class DDPGPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/ddpg", "Deep Deterministic Policy Gradient Actor-Critic predictor")

  protected:
    double gamma_;
    Projector *obs_projector_, *action_projector_;
    TensorFlowRepresentation *representation_;
    std::string  observation_, action_, value_, target_, critic_update_, actor_update_;

  public:
    DDPGPredictor() : gamma_(0.97), obs_projector_(NULL), action_projector_(NULL), representation_(NULL) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Predictor
    virtual void update(const Transition &transition) { throw Exception("Incremental update not supported"); }
    virtual void update(const std::vector<const Transition*> &transitions);
    virtual void finalize();
};

}

#endif /* GRL_NAF_PREDICTOR_H_ */
