/** \file snapping.h
 * \brief Predictor that snaps updates to grid centers header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2017-12-05
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

#ifndef GRL_SNAPPING_PREDICTOR_H_
#define GRL_SNAPPING_PREDICTOR_H_

#include <grl/predictor.h>
#include <grl/discretizer.h>
#include <grl/environments/observation.h>

namespace grl
{

/// Predictor that snaps updates to grid centers.
class SnappingPredictor : public Predictor
{
  public:
    TYPEINFO("predictor/snapping", "Snaps updates to grid centers")
    
  protected:
    Discretizer *discretizer_;
    ObservationModel *model_;
    Predictor *predictor_;
    
    Vector min_, max_, steps_, delta_;
    IndexVector stride_;
    
    size_t centers_;
    
  public:
    SnappingPredictor() : discretizer_(NULL), model_(NULL), predictor_(NULL), centers_(0) { }
  
    // From Configurable    
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Predictor
    virtual void update(const Transition &transition);
    virtual void finalize() { }
};

}

#endif /* GRL_SNAPPING_PREDICTOR_H_ */
