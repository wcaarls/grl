/** \file reinforce.h
 * \brief Monte-Carlo Policy Gradient predictor header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-05-09
 *
 * \copyright \verbatim
 * Copyright (c) 2017, Wouter Caarls
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

#ifndef GRL_REINFORCE_PREDICTOR_H_
#define GRL_REINFORCE_PREDICTOR_H_

#include <grl/predictor.h>
#include <grl/discretizer.h>
#include <grl/projector.h>
#include <grl/representation.h>

namespace grl
{

/// Monte-Carlo Policy Gradient predictor (REINFORCE).
class ReinforcePredictor : public Predictor
{
  public:
    TYPEINFO("predictor/reinforce", "Monte-Carlo Policy Gradient predictor")

  protected:
    double alpha_, gamma_;
    Projector *projector_;
    Representation *representation_;
    std::vector<Transition> transitions_;

  public:
    ReinforcePredictor() : alpha_(0.1), gamma_(0.97), projector_(NULL), representation_(NULL) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Predictor
    virtual void update(const Transition &transition);
    virtual void finalize();
};

}

#endif /* GRL_REINFORCE_PREDICTOR_H_ */
