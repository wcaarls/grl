/** \file sarsa.h
 * \brief SARSA and Expected SARSA predictors header file.
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

#ifndef GRL_SARSA_PREDICTOR_H_
#define GRL_SARSA_PREDICTOR_H_

#include <grl/configurable.h>
#include <grl/predictor.h>
#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/trace.h>
#include <grl/policy.h>
#include <grl/policies/q.h>
#include <grl/mapping.h>

namespace grl
{

/// Value function predictor using the taken action.
class SARSAPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/sarsa", "SARSA on-policy value function predictor")

  protected:
    double alpha_, gamma_, lambda_;
    Projector *projector_;
    Representation *representation_;
    Trace *trace_;

  public:
    SARSAPredictor() : alpha_(0.2), gamma_(0.97), lambda_(0.65), projector_(NULL), representation_(NULL), trace_(NULL) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Predictor
    virtual SARSAPredictor *clone() const;
    virtual void update(const Transition &transition);
    virtual void finalize();
};

/// Value function predictor using the distribution of a QPolicy.
class ExpectedSARSAPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/expected_sarsa", "Expected SARSA low-variance on-policy value function predictor")

  protected:
    double alpha_, gamma_, lambda_;
    Projector *projector_;
    Representation *representation_;
    QPolicy *policy_;
    Trace *trace_;

  public:
    // From Configurable
    ExpectedSARSAPredictor() : alpha_(0.2), gamma_(0.97), lambda_(0.65), projector_(NULL), representation_(NULL), policy_(NULL), trace_(NULL) { }
     
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Predictor
    virtual ExpectedSARSAPredictor *clone() const;
    virtual void update(const Transition &transition);
    virtual void finalize();
};

/// SARSA value function predictor with shaping rewards.
class ShapedSARSAPredictor : public SARSAPredictor
{
  public:
    TYPEINFO("predictor/shaped_sarsa", "SARSA predictor with shaping rewards")

  protected:
    Mapping *shaping_function_;

  public:
    ShapedSARSAPredictor() : SARSAPredictor(), shaping_function_(NULL) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Predictor
    virtual ShapedSARSAPredictor *clone() const;
    virtual void update(const Transition &transition);
    virtual void finalize();
};

}

#endif /* GRL_SARSA_PREDICTOR_H_ */
