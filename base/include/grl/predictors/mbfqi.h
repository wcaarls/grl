/** \file mbfqi.h
 * \brief Minibatch FQI predictor header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-01-02
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

#ifndef GRL_MBFQI_PREDICTOR_H_
#define GRL_MBFQI_PREDICTOR_H_

#include <grl/predictor.h>
#include <grl/mapping.h>
#include <grl/projector.h>
#include <grl/representation.h>

namespace grl
{

/// Fitted Q-iteration predictor.
class MBFQIPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/mbfqi", "Minibatch FQI predictor")
    
    struct CachedTransition
    {
      Transition transition;
      ProjectionPtr projection;
      double target;
      
      CachedTransition(const Transition &t = Transition()) : transition(t) { }
    };
    
  protected:
    double alpha_, gamma_;
    Mapping *target_;
    Projector *projector_;
    Representation *representation_;
    
    size_t max_samples_, update_interval_;
    std::vector<CachedTransition> transitions_;
    std::vector<size_t> indices_;
    size_t minibatch_size_, update_counter_;

  public:
    MBFQIPredictor() : alpha_(1.0), gamma_(0.97), target_(NULL), projector_(NULL), representation_(NULL), max_samples_(100000), update_interval_(10), minibatch_size_(64), update_counter_(0) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    virtual MBFQIPredictor &copy(const Configurable &obj);
    
    // From Predictor
    virtual void update(const Transition &transition);
    virtual void finalize();

  protected:
    virtual void rebuild();
};

}

#endif /* GRL_MBFQI_PREDICTOR_H_ */
