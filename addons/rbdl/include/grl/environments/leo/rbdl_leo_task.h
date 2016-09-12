/** \file leotask.h
 * \brief RBDL file for C++ description of Leo task.
 *
 * \author    Ivan Koryakovskiy <i.koryakovskiy@tudelft.nl>
 * \date      2016-06-30
 *
 * \copyright \verbatim
 * Copyright (c) 2016, Ivan Koryakovskiy
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
 
#ifndef GRL_RBDL_LEO_TASK_H_
#define GRL_RBDL_LEO_TASK_H_

#include <functional>
#include <grl/environment.h>
//???? used? #include <grl/environments/leohelper.h>
#include <grl/environments/rbdl.h>

namespace grl
{

enum RbdlLeoState
{
  rlsAnkleAngle,
  rlsKneeAngle,
  rlsHipAngle,
  rlsArmAngle,                    // might not be used

  rlsDofDim = rlsArmAngle + 1,

  rlsAnkleAngleRate = rlsDofDim,
  rlsKneeAngleRate,
  rlsHipAngleRate,
  rlsArmAngleRate,                // might not be used

  rlsTime,
  rlsRefRootZ,

  rlsLeftTipX,
  rlsLeftTipY,
  rlsLeftTipZ,

  rlsLeftHeelX,
  rlsLeftHeelY,
  rlsLeftHeelZ,

  rlsRootX,
  rlsRootY,
  rlsRootZ,

  rlsMass,

  rlsComX,
  rlsComY,
  rlsComZ,

  rlsComVelocityX,
  rlsComVelocityY,
  rlsComVelocityZ,

  rlsAngularMomentumX,
  rlsAngularMomentumY,
  rlsAngularMomentumZ,

  rlsStateDim = rlsAngularMomentumZ + 1
};

enum SquattingTaskState
{
  stsSquats = rlsStateDim,
  stsStateDim
};

class LeoSquattingTask : public Task
{
  public:
    TYPEINFO("task/leoSquattingFA", "Task specification for Leo squatting with a fixed arm")

  public:
    LeoSquattingTask() : timeout_(0), rand_init_(0), init_height_(0.28) { }

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);

    // From Task
    virtual LeoSquattingTask *clone() const;
    virtual void start(int test, Vector *state) const;
    virtual void observe(const Vector &state, Vector *obs, int *terminal) const;
    virtual void evaluate(const Vector &state, const Vector &action, const Vector &next, double *reward) const;
    virtual void report(std::ostream &os, const Vector &state) const;
    virtual bool invert(const Vector &obs, Vector *state) const;
    virtual Matrix rewardHessian(const Vector &state, const Vector &action) const;

  protected:
    virtual int failed(const Vector &state) const;

  protected:
    double timeout_, init_height_;
    int rand_init_;
    Vector target_obs_min_, target_obs_max_;
};

}

#endif /* GRL_RBDL_LEO_TASK_H_ */
