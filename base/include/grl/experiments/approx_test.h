/** \file approx_test.h
 * \brief Approximation test header file.
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

#ifndef GRL_APPROX_TEST_EXPERIMENT_H_
#define GRL_APPROX_TEST_EXPERIMENT_H_

#include <grl/projector.h>
#include <grl/representation.h>
#include <grl/experiment.h>

namespace grl
{

/// Approximation test experiment (supervised learning).
class ApproxTestExperiment : public Experiment
{
  public:
    TYPEINFO("experiment/approx_test", "Approximator test experiment (supervised learning)")

  protected:
    Projector *projector_;
    Representation *representation_;
    Mapping *mapping_;
    
    Vector min_, max_;
    size_t outputs_, train_samples_, test_samples_;
    std::string file_;

  public:
    ApproxTestExperiment() : projector_(NULL), representation_(NULL), mapping_(NULL), outputs_(1), train_samples_(1000), test_samples_(1000) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Experiment
    virtual ApproxTestExperiment *clone() const;
    virtual void run();
};

}

#endif /* GRL_APPROX_TEST_EXPERIMENT_H_ */
