/** \file model.h
 * \brief Model predictor header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-02-23
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

#ifndef GRL_MODEL_PREDICTOR_H_
#define GRL_MODEL_PREDICTOR_H_

#include <grl/predictor.h>
#include <grl/projector.h>
#include <grl/representation.h>

namespace grl
{

/// Model predictor.
class ModelPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/model")

  protected:
    Projector *projector_;
    Representation *representation_;
    
    int differential_;
    Vector wrapping_;

  public:
    ModelPredictor() : projector_(NULL), representation_(NULL), differential_(1) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Predictor
    virtual ModelPredictor *clone() const;
    virtual void update(const Transition &transition);
    virtual void finalize();
};

}

#endif /* GRL_MODEL_PREDICTOR_H_ */
