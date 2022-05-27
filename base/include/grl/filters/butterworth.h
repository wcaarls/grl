/** \file butterworth.h
 * \brief Butterworth filter definition.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2022-05-27
 *
 * \copyright \verbatim
 * Copyright (c) 2022, Wouter Caarls
 * Copyright (c) 2008, Erik Schuitema
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

#ifndef GRL_BUTTERWORTH_FILTER_H_
#define GRL_BUTTERWORTH_FILTER_H_

#include <grl/configurable.h>
#include <grl/utils.h>
#include <grl/grl.h>

namespace grl
{

/// Filters a signal using a Butterworth filter, up to 3rd order.
class ButterworthFilter : public Configurable
{
  public:
    TYPEINFO("filter/butterworth", "Third order Butterworth filter")

    int order_;
    double sampling_frequency_, cutoff_frequency_;
    Vector kout_, // kout_[0] is not used; it is the output
           kin_;
           
    std::vector<Vector> samplebuffer_,
                        filterbuffer_; // Element 0  is the most recent; element 1 is 1 update old, etc.
                        
    bool first_;

  public:
    ButterworthFilter() : order_(3), sampling_frequency_(1.), cutoff_frequency_(0.5), first_(true) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Filter
    virtual Vector filter(const Vector &sample);
    virtual void clear();
};

}

#endif /* GRL_BUTTERWORTH_FILTER_H_ */
