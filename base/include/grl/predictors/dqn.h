/** \file dqn.h
 * \brief DQN predictor header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2023-02-02
 *
 * \copyright \verbatim
 * Copyright (c) 2023, Wouter Caarls
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

#ifndef GRL_DQN_PREDICTOR_H_
#define GRL_DQN_PREDICTOR_H_

#include <grl/configurable.h>
#include <grl/predictor.h>
#include <grl/discretizer.h>
#include <grl/projector.h>
#include <grl/representation.h>

namespace grl
{

/// Q-learning predictor using action Q-vectors.
class DQNPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/dqn", "Deep Q-learning off-policy value function predictor")

  protected:
    double gamma_;
    Discretizer *discretizer_;
    Projector *projector_;
    Representation *representation_;

  public:
    DQNPredictor() : gamma_(0.97), discretizer_(NULL), projector_(NULL), representation_(NULL) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Predictor
    virtual void update(const Transition &transition)
    {
      Predictor::update(transition);
      std::vector<const Transition*> transitions = {&transition};
      update(transitions);
    }
    virtual void update(const std::vector<const Transition*> &transitions);
    virtual void finalize();
};

}

#endif /* GRL_DQN_PREDICTOR_H_ */
