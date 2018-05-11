/** \file naf.h
 * \brief NAF and Expected NAF predictors header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-07-18
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

/// Value function predictor using Normalized Advantage Features.
class NAFPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/naf", "Value function predictor using Normalized Advantage Features")

  protected:
    double gamma_;
    Projector *projector_;
    TensorFlowRepresentation *representation_;
    std::string input_, value_, target_, update_;

  public:
    NAFPredictor() : gamma_(0.97), projector_(NULL), representation_(NULL) { }
  
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
