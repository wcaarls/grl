/** \file ilqg.h
 * \brief ILQG solver header file.
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2015-10-07
 *
 * \copyright \verbatim
 * Copyright (c) 2013, Yuval Tassa
 * Copyright (c) 2015, Wouter Caarls
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the distribution
 *     * Neither the name of the University of Washington nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ILQG_SOLVER_H_
#define ILQG_SOLVER_H_

#include <Eigen/Eigen>
#include <grl/solver.h>
#include <grl/environments/observation.h>
#include <grl/policies/state_feedback.h>
#include <grl/signal.h>

namespace grl {

/// Iterative Linear Quadratic Gaussian trajectory optimizer
class ILQGSolver : public Solver
{
  public:
    TYPEINFO("solver/ilqg", "Iterative Linear Quadratic Gaussian trajectory optimizer");
    
  protected:
    typedef std::vector<Matrix> Matrix3D;
    enum RegularizationMethod {rmNone, rmState, rmControls};
  
    ObservationModel *model_;
    SampleFeedbackPolicy *policy_;
    MatrixSignal *trajectory_;
    size_t horizon_, maxiter_;
    double lambda_min_, lambda_max_, lambda_factor_, tolerance_;
    Vector stddev_;
    std::string regularization_;

    double t0_, step_;
    Matrix x_, u_;
    Matrix3D L_;

  public:
    ILQGSolver() : model_(NULL), policy_(NULL), trajectory_(NULL), horizon_(100), maxiter_(100), lambda_min_(1e-6), lambda_max_(1e10), lambda_factor_(1.6), tolerance_(1e-5), regularization_("state"), t0_(0), step_(0) { }

    // From Configurable 
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);
    virtual ILQGSolver &copy(const Configurable &obj);

    // From Solver
    virtual bool solve() { return false; }
    virtual bool solve(const Vector &x0);
    virtual bool resolve(double t, const Vector &xt);

  protected:
    bool forwardPass(const ColumnVector &x0, const Matrix &x, const Matrix &u, const Matrix3D &L, Matrix *xnew, Matrix *unew, RowVector *cnew) const;
    bool backwardPass(const Matrix3D &J, const Matrix3D &H, double lambda, Matrix *l, Matrix3D *L, RowVector *dV) const;
};

}

#endif /* ILQG_SOLVER_H_ */
