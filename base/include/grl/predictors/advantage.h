/** \file advantage.h
 * \brief Advantage learning predictor header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-10-05
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

#ifndef GRL_ADVANTAGE_PREDICTOR_H_
#define GRL_ADVANTAGE_PREDICTOR_H_

#include <grl/configurable.h>
#include <grl/predictor.h>
#include <grl/discretizer.h>
#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/trace.h>

namespace grl
{

/// Value function predictor using the best next action.
class QPredictor : public CriticPredictor
{
  public:
    TYPEINFO("predictor/critic/q", "Q-learning off-policy value function predictor")

  protected:
    double alpha_, gamma_, lambda_;
    Discretizer *discretizer_;
    Projector *projector_;
    Representation *representation_;
    Trace *trace_;
    
    std::vector<Vector> variants_;

  public:
    QPredictor() : alpha_(0.2), gamma_(0.97), lambda_(0.65), discretizer_(NULL), projector_(NULL), representation_(NULL), trace_(NULL) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Predictor
    virtual void finalize();

    // From CriticPredictor
    virtual double criticize(const Transition &transition, const Action &action);
};

/// Value function predictor that emphasizes within-state action-value differences.
class AdvantagePredictor : public CriticPredictor
{
  public:
    TYPEINFO("predictor/critic/advantage", "Advantage learning off-policy value function predictor")

  protected:
    double alpha_, gamma_, lambda_, kappa_;
    Discretizer *discretizer_;
    Projector *projector_;
    Representation *representation_;
    Trace *trace_;
    
    std::vector<Vector> variants_;

  public:
    AdvantagePredictor() : alpha_(0.2), gamma_(0.97), lambda_(0.65), kappa_(.2), discretizer_(NULL), projector_(NULL), representation_(NULL), trace_(NULL) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Predictor
    virtual void finalize();

    // From CriticPredictor
    virtual double criticize(const Transition &transition, const Action &action);
};

}

#endif /* GRL_ADVANTAGE_PREDICTOR_H_ */
