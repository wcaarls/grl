/** \file qv.h
 * \brief QV predictor header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-06-30
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

#ifndef GRL_QV_PREDICTOR_H_
#define GRL_QV_PREDICTOR_H_

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

/**
 * \brief Value function predictor using base state-values and state-action values.
 *
 * From Wiering, 2005,
 * "QV(\lambda)-learning: A New On-policy Reinforcement Learning Algorithm"
 */
class QVPredictor : public CriticPredictor
{
  public:
    TYPEINFO("predictor/critic/qv", "QV on-policy value function predictor")

  protected:
    double alpha_, beta_, gamma_, lambda_;
    
    Projector *v_projector_, *q_projector_;
    Representation *v_representation_, *q_representation_;
    Trace *trace_;

  public:
    QVPredictor() : alpha_(0.2), beta_(0.1), gamma_(0.97), lambda_(0.65), v_projector_(NULL), q_projector_(NULL), v_representation_(NULL), q_representation_(NULL), trace_(NULL) { }
  
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

#endif /* GRL_QV_PREDICTOR_H_ */
