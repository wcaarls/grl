/** \file ilqg.cpp
 * \brief ILQG solver source file.
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

#include <grl/solvers/ilqg.h>

using namespace grl;

REGISTER_CONFIGURABLE(ILQGSolver)

void ILQGSolver::request(ConfigurationRequest *config)
{
  config->push_back(CRP("horizon", "Horizon", (int)horizon_));
  config->push_back(CRP("iterations", "Maximum number of iterations", (int)maxiter_));
  config->push_back(CRP("stddev", "Standard deviation of initial random action sequence", stddev_));
  
  std::vector<std::string> options;
  options.push_back("none");
  options.push_back("state");
  options.push_back("controls");
  config->push_back(CRP("regularization", "Regularization method", regularization_, CRP::Configuration, options));

  config->push_back(CRP("model", "observation_model", "Observation model", model_));
  config->push_back(CRP("policy", "mapping/policy/sample_feedback", "Sample feedback policy to adjust", policy_));

  config->push_back(CRP("trajectory", "signal/matrix", "Predicted trajectory", CRP::Provided));
}

void ILQGSolver::configure(Configuration &config)
{
  model_ = (ObservationModel*)config["model"].ptr();
  policy_ = (SampleFeedbackPolicy*)config["policy"].ptr();
  
  horizon_ = config["horizon"];
  stddev_ = config["stddev"].v();
  maxiter_ = config["iterations"];
  regularization_ = config["regularization"].str();
  
  if (!stddev_.size())
    throw bad_param("solver/ilqg:stddev");

  trajectory_ = new MatrixSignal();
  config.set("trajectory", trajectory_);
}

void ILQGSolver::reconfigure(const Configuration &config)
{
}

ILQGSolver &ILQGSolver::copy(const Configurable &obj)
{
  const ILQGSolver &is = dynamic_cast<const ILQGSolver&>(obj);
  
  t0_ = is.t0_;
  step_ = is.step_;
  x_ = is.x_;
  u_ = is.u_;
  L_ = is.L_;

  return *this;
}

bool ILQGSolver::solve(const Vector &x0)
{
  // Solve
  return resolve(0, x0);
}

bool ILQGSolver::resolve(double t, const Vector &xt)
{
  size_t n = xt.size(),      // Number of state dimensions
         m = stddev_.size(), // Number of action dimensions
         N = horizon_;       // Horizon length

  if (!step_)
  {
    // Retrieve step size (assuming it's constant)
    Vector next;
    double reward;
    int terminal;
    step_ = model_->step(xt, stddev_, &next, &reward, &terminal);
    
    if (!step_)
      return false;
  }

  // Reset on new episode
  if (!t)
    u_ = Matrix();

  // On first call (or aborted previous attempt), construct random action sequence
  if (u_.cols() != N)
  {
    CRAWL("Generating initial random action sequence");
  
    // Cold start
    t0_ = t;
    x_ = Matrix();
    L_ = Matrix3D();

    // Generate initial action sequence
    u_ = Matrix::Zero(stddev_.size(), horizon_);

    for (size_t ii=0; ii < u_.rows(); ++ii)
      for (size_t jj=0; jj < u_.cols(); ++jj)
        u_(ii, jj) = RandGen::getNormal(0, stddev_[ii]);
  }    

  size_t start = round((t-t0_)/step_);
  
  if (start > 0)
  {
    CRAWL("Starting from previous solution shifted by " << start << " steps");
    
    size_t Nx = std::max(std::min((size_t)x_.cols(), N), start);
    size_t NL = std::max(std::min((size_t)L_.size(), N), start);
    grl_assert(u_.cols() == N);
  
    // Throw away solution before t
    u_.leftCols(N-start) = u_.rightCols(N-start).eval();
    x_ = x_.block(0, start, n, Nx-start).eval();
    for (size_t ii=0; ii < NL-start; ++ii)
      L_[ii].swap(L_[ii+start]);
    L_.resize(NL-start);
     
    // Pad actions with last action
    for (size_t ii=N-start; ii < N; ++ii)
      u_.col(ii) = u_.col(N-1);

    trajectory_->set(x_);
  }
  
  t0_ = t;

  Matrix x,       // States,         n x (N+1)
         u;       // Actions,        m x N
  Matrix3D L;     // Feedback gains, m x n x N
  RowVector cost; // Costs,          1 x (N+1)

  // Roll out initial trajectory
  if (!forwardPass(xt.transpose(), x_, u_, L_, &x, &u, &cost))
  {
    WARNING("Could not complete initial forward pass");
    u_ = Matrix();
    return false;
  }

  Matrix3D J(N+1), // Dynamics and cost Jacobian, (n+1) x (n+m) x (N+1)
           H(N+1); // Cost Hessian,               (n+m) x (n+m) x (N+1)
  Matrix l;        // Action gradient, m x N
  RowVector dV;    // 1 x 2  
  double lambda = 1, dlambda = 1; // Regularization parameter and scaling factor
  bool accepted = true;
  
  for (size_t iteration=0; iteration < maxiter_; ++iteration)
  {
    // ======
    // ====== STEP 1: differentiate dynamics and cost along new trajectory
    // ======
    
    if (accepted)
    {
      // NOTE: May be done in parallel
      for (size_t ss=0; ss < N; ++ss)
      {
        J[ss] = model_->jacobian(x.col(ss).transpose(), u.col(ss).transpose());
        J[ss].row(n) = -J[ss].row(n);
        H[ss] = -model_->rewardHessian(x.col(ss).transpose(), u.col(ss).transpose());
      }
    
      // Final cost Jacobian and Hessian. In our case, just the last state cost with action zero.
      J[N] = model_->jacobian(x.col(N).transpose(), ConstantVector(m, 0.).transpose());
      J[N].row(n) = -J[N].row(n);
      H[N] = -model_->rewardHessian(x.col(N).transpose(), ConstantVector(m, 0.).transpose());
      
      accepted = false;
    }

    // ======
    // ====== STEP 2: backward pass, compute optimal control law and cost-to-go
    // ======

    // NOTE: May be done in parallel for THREAD_COUNT lambdas, choosing smallest one that succeeds
    while (!backwardPass(J, H, lambda, &l, &L, &dV))
    {
      // Increase regularization parameter
      dlambda   = fmax(dlambda * lambda_factor_, lambda_factor_);
      lambda    = fmax(lambda * dlambda, lambda_min_);
      
      CRAWL("Increased regularization to " << lambda << " (backward pass)");
      if (lambda > lambda_max_)
      {
        WARNING("Regularization limit reached while applying backward pass");
        return false;
      }
    }
    
    // check for termination due to small gradient
    double g_norm = (abs(l.array()) / (abs(u.array())+1)).rowwise().maxCoeff().mean();
    if (g_norm < tolerance_ && lambda < 1e-5)
    {
      TRACE("iLQG converged after " << iteration << " iterations (gradient)");
      break;
    }
    
    // ======
    // ====== STEP 3: line-search to find new control sequence, trajectory, cost
    // ======
    
    Matrix xnew, unew;
    RowVector costnew;
    double total_cost = cost.sum();
    
    /// NOTE: May be done in parallel for all alphas, choosing one with highest dcost
    for (size_t ff = 0; ff < 8; ++ff)
    {
      double alpha = pow(10, -0.4286*ff);
    
      if (!forwardPass(x.col(0), x, u+l*alpha, L, &xnew, &unew, &costnew))
      {
        WARNING("Could not complete forward pass");
        return false;
      }
      
      double dcost = total_cost - costnew.sum();
      double expected = -alpha*(dV[0] + alpha*dV[1]), z;
      
      if (expected > 0)
      {
        z = dcost/expected;
      }
      else
      {
        z = sign(dcost);
        TRACE("non-positive expected reduction: should not occur");
      }
      
      if (z > 0)
      {
        accepted = true;
        break;
      }
    }
    
    // ======
    // ====== STEP 4: accept (or not)
    // ======
    
    if (accepted)
    {
      u.swap(unew);
      x.swap(xnew);
      cost.swap(costnew);
      
      double costsum = cost.sum();

      CRAWL(iteration << ": accepted; cost " << costsum);
      
      if ((total_cost - costsum) < tolerance_)
      {
        TRACE("iLQG converged after " << iteration << " iterations (cost change)");
        break;
      }
      
      // Decrease regularization parameter
      dlambda   = fmin(dlambda / lambda_factor_, 1./lambda_factor_);
      lambda    = lambda * dlambda * (lambda > lambda_min_);

      // Save for next call
      x_ = x;
      u_= u;
      L_ = L;
      
      trajectory_->set(x_);
      
      policy_->clear();
      for (size_t ii=0; ii < N; ++ii)
        policy_->push(SampleFeedbackPolicy::Sample(x_.col(ii).transpose(), u_.col(ii).transpose(), L_[ii]));
    }
    else
    {
      // Increase regularization parameter
      dlambda   = fmax(dlambda * lambda_factor_, lambda_factor_);
      lambda    = fmax(lambda * dlambda, lambda_min_);
      
      CRAWL(iteration << ": rejected; increased regularization to " << lambda << " (cost improvement)");
      if (lambda > lambda_max_)
      {
        WARNING("Regularization limit reached while improving cost");
        return false;
      }
    }
  }
  
  return true;
}

bool ILQGSolver::forwardPass(const ColumnVector &x0, const Matrix &x, const Matrix &u, const Matrix3D &L, Matrix *xnew, Matrix *unew, RowVector *cnew) const
{
  size_t n = x0.rows(), // Number of state dimensions
         m = u.rows(),  // Number of action dimensions
         N = u.cols();  // Horizon length

  xnew->resize(n, N+1);
  xnew->col(0) = x0;
  unew->resize(m, N);
  cnew->resize(N+1);
  
  for (size_t i = 0; i < N; ++i)
  {
    // Feedforward
    unew->col(i) = u.col(i);
    
    // Feedback, if available
    if (L.size() > i)
    {
      ColumnVector dx = xnew->col(i) - x.col(i);
      unew->col(i) = unew->col(i) + L[i]*dx;
    }
    
    Vector next;
    double reward;
    int terminal;
    
    model_->step(xnew->col(i).transpose(), unew->col(i).transpose(), &next, &reward, &terminal);
    
    if (!next.size())
    {
      TRACE("Model prediction failed at step " << i << "(state " << xnew->col(i).transpose() << ", action " << unew->col(i).transpose() << ")");
      return false;
    }
    
    xnew->col(i+1).transpose() = next;
    (*cnew)[i] = -reward;
  }
  
  // No final cost
  (*cnew)[N] = 0.;

  return true;
}

bool ILQGSolver::backwardPass(const Matrix3D &J, const Matrix3D &H, double lambda, Matrix *l, Matrix3D *L, RowVector *dV) const
{
  // J is the Dynamics and cost Jacobian, (n+1) x (n+m) x (N+1)
  // H is the Cost Hessian,               (n+m) x (n+m) x (N+1)

  size_t m = H[0].rows()-J[0].rows()+1, // Number of action dimensions
         n = H[0].rows()-m,             // Number of state dimensions
         N = H.size()-1;                // Horizon length
         
  RegularizationMethod regularization;
  if (regularization_ == "state")
    regularization = rmState;
  else if (regularization_ == "controls")
    regularization = rmControls;
  else
    regularization = rmNone;
  
  l->resize(m, N);
  L->resize(N);
  
  Matrix Vx(n, N+1);
  Matrix3D Vxx(N+1);
  *dV = RowVector::Zero(2);
  
  // Value function at the horizon is just immediate cost
  Vx.col(N) = J[N].row(n).head(n).transpose();
  Vxx[N] = H[N].topLeftCorner(n, n);

  // Work backwards to starting state
  for (int i=N-1; i >= 0; --i)
  {
    Matrix fx = J[i].topLeftCorner(n, n),
           fu = J[i].topRightCorner(n, m);
    
    // Qu  = cu(:,i)      + fu(:,:,i)'*Vx(:,i+1);
    Matrix Qu = J[i].row(n).tail(m).transpose() + fu.transpose() * Vx.col(i+1);
    
    // Qx  = cx(:,i)      + fx(:,:,i)'*Vx(:,i+1);
    Matrix Qx = J[i].row(n).head(n).transpose() + fx.transpose() * Vx.col(i+1);
    
    // Qux = cxu(:,:,i)'  + fu(:,:,i)'*Vxx(:,:,i+1)*fx(:,:,i);
    Matrix Qux = H[i].topRightCorner(n, m).transpose() + fu.transpose() * Vxx[i+1] * fx;
       
    // Quu = cuu(:,:,i)   + fu(:,:,i)'*Vxx(:,:,i+1)*fu(:,:,i);
    Matrix Quu = H[i].bottomRightCorner(m, m) + fu.transpose() * Vxx[i+1] * fu;
       
    // Qxx = cxx(:,:,i)   + fx(:,:,i)'*Vxx(:,:,i+1)*fx(:,:,i);
    Matrix Qxx = H[i].topLeftCorner(n, n) + fx.transpose() * Vxx[i+1] * fx;
       
    // Vxx_reg = (Vxx(:,:,i+1) + lambda*eye(n)*(regType == 2));
    Matrix Vxx_reg = Vxx[i+1] + lambda*Matrix::Identity(n, n)*(regularization==rmState);
    
    // Qux_reg = cxu(:,:,i)'   + fu(:,:,i)'*Vxx_reg*fx(:,:,i);
    Matrix Qux_reg = H[i].topRightCorner(n, m).transpose() + fu.transpose() * Vxx_reg * fx;
       
    // QuuF = cuu(:,:,i)  + fu(:,:,i)'*Vxx_reg*fu(:,:,i) + lambda*eye(m)*(regType == 1);
    Matrix QuuF = H[i].bottomRightCorner(m, m) + fu.transpose() * Vxx_reg * fu + lambda*Matrix::Identity(m, m)*(regularization==rmControls);
    
    // [R,d] = chol(QuuF);
    Eigen::LLT<Matrix> chol(QuuF);
    if (chol.info() != Eigen::Success)
    {
      TRACE("Cholesky failed at step " << i);
      return false;
    }

    // kK = -R\(R'\[Qu Qux_reg]);
    Matrix QuQux(m, n+1); QuQux << Qu, Qux_reg;
    Matrix kK = -chol.matrixU().solve(chol.matrixL().solve(QuQux));

    // k_i = kK(:,1);
    ColumnVector k_i = kK.col(0);
    
    // K_i = kK(:,2:n+1);
    Matrix K_i = kK.rightCols(n);
    
    // dV          = dV + [k_i'*Qu  .5*k_i'*Quu*k_i];
    Matrix temp = k_i.transpose()*Qu;
    *dV = *dV + VectorConstructor(temp(0, 0), .5*(k_i.transpose()*Quu*k_i)[0]).matrix();
    
    // Vx(:,i)     = Qx  + K_i'*Quu*k_i + K_i'*Qu  + Qux'*k_i;
    Vx.col(i) = Qx + K_i.transpose()*Quu*k_i + K_i.transpose()*Qu + Qux.transpose()*k_i;
    
    // Vxx(:,:,i)  = Qxx + K_i'*Quu*K_i + K_i'*Qux + Qux'*K_i;
    Vxx[i] = Qxx + K_i.transpose()*Quu*K_i + K_i.transpose()*Qux + Qux.transpose()*K_i;
    
    // Vxx(:,:,i)  = .5*(Vxx(:,:,i) + Vxx(:,:,i)');
    Vxx[i] = .5*(Vxx[i] + Vxx[i].transpose());
    
    // k(:,i)      = k_i;   
    l->col(i) = k_i;
    
    // K(:,:,i)    = K_i;   
    (*L)[i] = K_i;
  }

  return true;
}
