/** \file softmax.h
 * \brief Softmax sampler header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-07-01
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

#ifndef GRL_SOFTMAX_SAMPLER_H_
#define GRL_SOFTMAX_SAMPLER_H_

#include <grl/sampler.h>
#include <grl/utils.h>

namespace grl
{

/// Softmax (Gibbs/Boltzmann) sampler
class SoftmaxSampler : public Sampler
{
  public:
    TYPEINFO("sampler/softmax", "Softmax (Gibbs/Boltzmann) sampler")

  protected:
    double tau_;

  public:
    SoftmaxSampler() : tau_(1.) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
  
    // From Sampler
    virtual SoftmaxSampler *clone();
    virtual size_t sample(const Vector &values, TransitionType &tt) const;
    virtual void distribution(const Vector &values, Vector *distribution) const;
};

}

#endif /* GRL_SOFTMAX_SAMPLER_H_ */
