/** \file ggq.h
 * \brief Greedy-GQ predictor header file.
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

#ifndef GRL_GGQ_PREDICTOR_H_
#define GRL_GGQ_PREDICTOR_H_

#include <grl/configurable.h>
#include <grl/predictor.h>
#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/trace.h>
#include <grl/policy.h>
#include <grl/policies/q.h>
#include <grl/sampler.h>

namespace grl
{

/// Off-policy value function predictor that converges with function approximation
class GGQPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/ggq", "Greedy-GQ off-policy value function predictor")

  protected:
    double alpha_, eta_, gamma_;
    Projector *projector_;
    Representation *representation_;
    Policy *policy_;

  public:
    GGQPredictor() : alpha_(0.2), eta_(0.01), gamma_(0.97), projector_(NULL), representation_(NULL), policy_(NULL) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Predictor
    virtual void update(const Transition &transition);
    virtual void finalize();
};

}

#endif /* GRL_GGQ_PREDICTOR_H_ */
