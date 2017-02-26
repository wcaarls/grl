/** \file lqr.h
 * \brief LQR solver header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-08-26
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

#ifndef GRL_LQR_SOLVER_H_
#define GRL_LQR_SOLVER_H_

#include <Eigen/Eigen>

#include <grl/solver.h>
#include <grl/environments/observation.h>
#include <grl/policies/state_feedback.h>

namespace grl
{

/// Linear Quadratic Regulator solver
class LQRSolver : public Solver
{
  public:
    TYPEINFO("solver/lqr", "Linear Quadratic Regulator solver")

  protected:
    ObservationModel *model_;
    StateFeedbackPolicy *policy_;
    Vector operating_state_, operating_action_;

  public:
    LQRSolver() : model_(NULL), policy_(NULL) { }
  
    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Solver
    virtual LQRSolver *clone() const;
    virtual bool solve();
    
  protected:
    virtual int solveDARE(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R, Matrix *X) const;
};

}

#endif /* GRL_LQR_SOLVER_H_ */
