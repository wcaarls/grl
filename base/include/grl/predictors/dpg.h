/** \file dpg.h
 * \brief Deterministic policy gradient predictor header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-12-26 
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Wouter Caarls
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

#ifndef GRL_DPG_PREDICTOR_H_
#define GRL_DPG_PREDICTOR_H_

#include <grl/configurable.h>
#include <grl/predictor.h>
#include <grl/projector.h>
#include <grl/representation.h>

namespace grl
{

/// Silver et al., "Deterministic Policy Gradient Algorithms", ICML 2014
class DPGPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/dpg", "Deterministic policy gradient predictor")
    enum PolicyTarget {OnPolicyTarget, OffPolicyTarget};

  protected:
    double alpha_, beta_v_, beta_a_, gamma_, lambda_;
    Projector *projector_;
    Representation *advantage_representation_, *critic_representation_, *actor_representation_;
    Trace *critic_trace_;
    std::string target_;
    PolicyTarget policy_target_;

  public:
    DPGPredictor() : alpha_(0.2), beta_v_(0.1), beta_a_(0.01), gamma_(0.97), lambda_(0.65), projector_(NULL), advantage_representation_(NULL), critic_representation_(NULL), actor_representation_(NULL), critic_trace_(NULL), target_("on-policy"), policy_target_(OnPolicyTarget) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Predictor
    virtual void update(const Transition &transition);
    virtual void finalize();
};

}

#endif /* GRL_DPG_PREDICTOR_H_ */
