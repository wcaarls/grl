/** \file compass_walker.h
 * \brief Compass walker environment header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-03-14
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

#ifndef GRL_COMPASS_WALKER_ENVIRONMENT_H_
#define GRL_COMPASS_WALKER_ENVIRONMENT_H_

#include <grl/environment.h>
#include <grl/environments/compass_walker/SWModel.h>

namespace grl
{

class CompassWalker
{
  public:
    enum stateIndex { siStanceLegAngle, siHipAngle, siStanceLegAngleRate, siHipAngleRate,
                      siStanceLegChanged, siStanceFootX, siLastHipX, siTime, siPrevTime };
    enum stateSize  { ssStateSize = siPrevTime+1};
};

// Compass (simplest) walker model.
class CompassWalkerModel : public Model
{
  public:
    TYPEINFO("model/compass_walker", "Simplest walker model from Garcia et al.")
    
  protected:
    double tau_;
    size_t steps_;
    CSWModel model_;
    
  public:
    CompassWalkerModel() : tau_(0.2), steps_(20) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Model
    virtual CompassWalkerModel *clone() const;
    virtual double step(const Vector &state, const Vector &action, Vector *next) const;
};

// Walk forward task for compass walker.
class CompassWalkerWalkTask : public Task
{
  public:
    TYPEINFO("task/compass_walker/walk", "Compass walker walking task")
  
  protected:
    double T_;
    double initial_state_variation_;

  public:
    CompassWalkerWalkTask() : T_(100), initial_state_variation_(0.2){ }
    
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    
    // From Task
    virtual CompassWalkerWalkTask *clone() const;
    virtual void start(int test, Vector *state) const;
    virtual void observe(const Vector &state, Vector *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const;
    virtual bool invert(const Vector &obs, Vector *state) const;
};

// Walk forward with a reference trajectory task for compass walker.
class CompassWalkerVrefTask : public CompassWalkerWalkTask
{
  public:
    TYPEINFO("task/compass_walker/vref", "Compass walker tracking velocity task")

  protected:
    double vref_, vref_2var_;

  public:
    CompassWalkerVrefTask() : vref_(0.1) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);

    // From Task
    virtual void evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const;
};

}

#endif /* GRL_COMPASS_WALKER_ENVIRONMENT_H_ */
