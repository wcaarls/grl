/** \file split.h
 * \brief Split discretizer header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-02-03
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

#ifndef GRL_SPLIT_DISCRETIZER_H_
#define GRL_SPLIT_DISCRETIZER_H_

#include <grl/configurable.h>
#include <grl/discretizer.h>

namespace grl
{

/**
 * \brief Split discretizer
 *
 * Combines two discretizers by outputting the union of both discretizations.
 * When identify is set, an extra dimension is added at the start or end to
 * indicate which discretization is active.
 */
class SplitDiscretizer : public Discretizer
{
  public:
    TYPEINFO("discretizer/split", "Compound discretizer")

  protected:
    std::vector<Discretizer*> discretizer_;
    size_t idxsize_, ressize_;
    int identify_;

  public:
    SplitDiscretizer() : discretizer_(2), idxsize_(0), ressize_(0), identify_(-1)
    {
      discretizer_[0] = discretizer_[1] = NULL;
    }
  
    // From Configurable
    virtual void request(const std::string &role, ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Discretizer
    virtual iterator begin(const Vector &point) const;
    virtual size_t size(const Vector &point) const;
    virtual void inc(iterator *it) const;
    virtual Vector get(const iterator &it) const;
    virtual Vector at(const Vector &point, size_t idx) const;
};

}

#endif /* GRL_SPLIT_DISCRETIZER_H_ */
