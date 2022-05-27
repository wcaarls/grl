/** \file filter.h
 * \brief Generic filter definition.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2022-05-27
 *
 * \copyright \verbatim
 * Copyright (c) 2022, Wouter Caarls
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

#ifndef GRL_FILTER_H_
#define GRL_FILTER_H_

#include <grl/configurable.h>
#include <grl/utils.h>
#include <grl/grl.h>

namespace grl
{

/// Filters a signal
class Filter : public Configurable
{
  public:
    virtual ~Filter() { }
    
    /**
    * \brief Add a sample to the filter
    * \param sample - Value at current timestep
    * \return Filtered value
    */
    virtual Vector filter(const Vector &sample) = 0;
    
    /// Clear the filter.
    virtual void clear();
};

}

#endif /* GRL_FILTER_H_ */
