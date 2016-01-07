/** \file swimmer.h
 * \brief Swimmer environment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2016-01-06
 *
 * \copyright \verbatim
 * Copyright (c) 2007, Yuval Tassa
 * Copyright (c) 2013, RLPy http://acl.mit.edu/RLPy
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
 
#ifndef GRL_SWIMMER_ENVIRONMENT_H_
#define GRL_SWIMMER_ENVIRONMENT_H_

#include <grl/environment.h>

namespace grl
{

/// Swimmer dynamics, based on Remi Coulom's code.
class SwimmerDynamics : public Dynamics
{
  public:
    TYPEINFO("dynamics/swimmer", "Coulom's swimmer dynamics")
  
  protected:
    int segments_;
  
    ColumnVector masses_, lengths_, inertia_;
    Matrix P_, U_, G_;
  
  public:
    SwimmerDynamics() : segments_(2) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Dynamics
    virtual SwimmerDynamics *clone() const;
    virtual void eom(const Vector &state, const Vector &action, Vector *xd) const;
    
  protected:
    Matrix v1Mv2(const ColumnVector &v1, const Matrix &M, const ColumnVector &v2) const;
};

/// Swimmer reaching task.
class SwimmerReachingTask : public Task
{
  public:
    TYPEINFO("task/swimmer/reaching", "Swimmer reaching task")
  
  protected:
    double T_, randomization_;
    int segments_;
    Matrix P_;
    
  public:
    SwimmerReachingTask() : T_(20), randomization_(0.), segments_(2) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Task
    virtual SwimmerReachingTask *clone() const;
    virtual void start(int test, Vector *state) const;
    virtual void observe(const Vector &state, Vector *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const;
    virtual bool invert(const Vector &obs, Vector *state) const;
    virtual Matrix rewardHessian(const Vector &state, const Vector &action) const;
};

}

#endif /* GRL_SWIMMER_ENVIRONMENT_H_ */
