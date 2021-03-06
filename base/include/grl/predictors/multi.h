/** \file multi.h
 * \brief Multi predictor header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-09-01
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

#ifndef GRL_MULTI_PREDICTOR_H_
#define GRL_MULTI_PREDICTOR_H_

#include <grl/predictor.h>

namespace grl
{

/// Multi predictor.
class MultiPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/multi", "Updates multiple predictors")

  protected:
    TypedConfigurableList<Predictor> predictor_;

  public:
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Predictor
    virtual void update(const Transition &transition);
    virtual void update(const std::vector<const Transition*> &transitions);
    virtual void finalize();
};

}

#endif /* GRL_MULTI_PREDICTOR_H_ */
