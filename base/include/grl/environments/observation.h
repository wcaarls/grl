/** \file observation.h
 * \brief Observation model header file.
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

#ifndef GRL_OBSERVATION_MODEL_H_
#define GRL_OBSERVATION_MODEL_H_

#include <grl/environment.h>
#include <grl/projector.h>
#include <grl/representation.h>

namespace grl
{

/// Model that works on observations instead of states.
class ObservationModel : public Configurable
{
  public:
    virtual ObservationModel *clone() const = 0;
    virtual double step(const Vector &obs, const Vector &action, Vector *next, double *reward, int *terminal) const = 0;
    virtual Matrix jacobian(const Vector &obs, const Vector &action) const;
};

/// Observation model that is all given beforehand.
class FixedObservationModel : public ObservationModel
{
  public:
    TYPEINFO("observation_model/fixed", "Observation model based on known state transition model")
    
  protected:
    Model *model_;
    Task *task_;
    
  public:
    FixedObservationModel() : model_(NULL), task_(NULL) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From ObservationModel
    virtual FixedObservationModel *clone() const;
    virtual double step(const Vector &obs, const Vector &action, Vector *next, double *reward, int *terminal) const;
};

/// Observation model that is all learned
class ApproximatedObservationModel : public ObservationModel
{
  public:
    TYPEINFO("observation_model/approximated", "Observation model based on observed transitions")
    
  protected:
    Projector *projector_;
    Representation *representation_;
    
    int differential_;
    Vector wrapping_, observation_min_, observation_max_;
    double stddev_limit_;
    
    double tau_;
    
  public:
    ApproximatedObservationModel() : projector_(NULL), representation_(NULL), differential_(1), stddev_limit_(1.), tau_(0.) { }
    
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From ObservationModel
    virtual ApproximatedObservationModel *clone() const;
    virtual double step(const Vector &obs, const Vector &action, Vector *next, double *reward, int *terminal) const;
};

/// Observation model in which the reward is given and the model is learned
class FixedRewardObservationModel : public ApproximatedObservationModel
{
  public:
    TYPEINFO("observation_model/fixed_reward", "Observation model based on observed transitions but known task")
    
  protected:
    Task *task_;
    
  public:
    FixedRewardObservationModel() : task_(NULL) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From ObservationModel
    virtual FixedRewardObservationModel *clone() const;
    virtual double step(const Vector &obs, const Vector &action, Vector *next, double *reward, int *terminal) const;
};

}

#endif /* GRL_OBSERVATION_MODEL_H_ */
