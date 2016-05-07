/** \file lqr.cpp
 * \brief LQR solver source file.
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

#include <grl/solvers/lqr.h>

#define EPS 0.001

using namespace grl;

REGISTER_CONFIGURABLE(LQRSolver)

void LQRSolver::request(ConfigurationRequest *config)
{
  config->push_back(CRP("operating_state", "Operating state around which to linearize", operating_state_));
  config->push_back(CRP("operating_action", "Operating action around which to linearize", operating_action_));

  config->push_back(CRP("model", "observation_model", "Observation model", model_));
  config->push_back(CRP("policy", "policy/parameterized/state_feedback", "State feedback policy to adjust", policy_));
}

void LQRSolver::configure(Configuration &config)
{
  model_ = (ObservationModel*)config["model"].ptr();
  policy_ = (StateFeedbackPolicy*)config["policy"].ptr();
  operating_state_ = config["operating_state"].v();
  operating_action_ = config["operating_action"].v();
  
  if (!operating_state_.size())
    throw bad_param("solver/lqr:operating_state");

  if (!operating_action_.size())
    throw bad_param("solver/lqr:operating_action");
    
  if (policy_->size() != operating_state_.size()*operating_action_.size())
  {
    ERROR("Policy doesn't have the right size. Expected: " << operating_state_.size()*operating_action_.size() << ", got " << policy_->size());
    throw bad_param("solver/lqr:{policy,operating_state,operating_action}");
  }
}

void LQRSolver::reconfigure(const Configuration &config)
{
}

LQRSolver *LQRSolver::clone() const
{
  return new LQRSolver(*this);
}

bool LQRSolver::solve()
{
  Matrix J = model_->jacobian(operating_state_, operating_action_);
  Matrix H = model_->rewardHessian(operating_state_, operating_action_);
  
  if (J.size() && H.size())
  {
    Matrix Q = -H.block(0, 0, operating_state_.size(), operating_state_.size());
    Matrix R = -H.block(operating_state_.size(), operating_state_.size(), operating_action_.size(), operating_action_.size());
  
    Matrix A(operating_state_.size(), operating_state_.size()), B(operating_state_.size(), operating_action_.size());
    for (size_t rr=0; rr < operating_state_.size(); ++rr)
    {
      for (size_t cc=0; cc < operating_state_.size(); ++cc)
        A(rr, cc) = J(rr, cc);
      for (size_t cc=0; cc < operating_action_.size(); ++cc)
        B(rr, cc) = J(rr, operating_state_.size()+cc);
    }

    TRACE("State jacobian:\n" << A);
    TRACE("Action jacobian:\n" << B);

    // Solve discrete-time algebraic Riccati equation
    Matrix X;
    int res = solveDARE(A, B, Q, R, &X);
    
    if (res != 0)
    {
      WARNING("Could not solve DARE: error " << res);
      return false;
    }
    
    // Compute feedback gain matrix
    Matrix L = (B.transpose()*X*B+R).inverse()*(B.transpose()*X*A);
    
    if (std::isfinite(L.array().sum()))
    {
      TRACE("Feedback gain matrix:\n" << L);
      
      for (size_t ii=0; ii < (size_t)L.cols(); ++ii)
        for (size_t oo=0; oo < (size_t)L.rows(); ++oo)
          policy_->params()[ii*(size_t)L.rows()+oo] = L(oo, ii);
    }
    else
    {
      WARNING("Calculated gain matrix contains infinities");
      return false;
    }
  }
  else
  {
    WARNING("Could not determine gain matrix");
    return false;
  }
    
  return true;
}

#define SSIZE 4096
extern "C" void sb02od_(char *DICO, char *JOBB, char *FACT, char *UPLO,
                        char *JOBL, char *SORT, int *N, int *M, int *P,
                        double *A, int *LDA, double *B, int *LDB,
                        double *Q, int *LDQ, double *R, int *LDR,
                        double *L, int *LDL, double *RCOND, double *X,
                        int *LDX, double *ALFAR, double *ALFAI,
                        double *BETA, double *S, int *LDS, double *T,
                        int *LDT, double *U, int *LDU, double *TOL,
                        int *IWORK, double *DWORK, int *LDWORK, int *BWORK,
                        int *INFO);

int LQRSolver::solveDARE(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R, Matrix *X) const
{
#ifdef WITH_SLICOT
  char DICO = 'D';
  char JOBB = 'B';
  char FACT = 'N';
  char UPLO = 'U';
  char JOBL = 'Z';
  char SORT = 'S';
  int N = A.cols();
  int M = B.cols();
  int P = 0;
  double L[1];
  int LDL=1;
  
  double RCOND;
  double ALFAR[SSIZE];
  double ALFAI[SSIZE];
  double BETA[SSIZE];
  double S[SSIZE];
  int LDS=2*N+M;
  double T[SSIZE];
  int LDT=2*N+M;
  double U[SSIZE];
  int LDU=2*N;
  double TOL=1e-5;
  int IWORK[SSIZE];
  double DWORK[SSIZE];
  int LDWORK=SSIZE;
  int BWORK[SSIZE];
  int INFO=0;
  
  X->resize(N, N);
  
  sb02od_(&DICO, &JOBB, &FACT, &UPLO, &JOBL, &SORT, &N, &M, &P, (double*)A.data(),
          &N, (double*)B.data(), &N, (double*)Q.data(), &N, (double*)R.data(), &M, L, &LDL, &RCOND, (double*)X->data(),
          &N, ALFAR, ALFAI, BETA, S, &LDS, T, &LDT, U,
          &LDU, &TOL, IWORK, DWORK, &LDWORK, BWORK, &INFO);

  return INFO;
#else
  Matrix At = A.transpose(), Bt = B.transpose();
  *X = Q;

  double d = EPS;
  for (size_t ii=0; ii < 1000 && d >= EPS; ++ii)
  {
    Matrix Xp = *X;
    
    *X = Q + At*(*X)*A - At*(*X)*B*(Bt*(*X)*B+R).inverse()*Bt*(*X)*A;
    d = (*X - Xp).array().abs().sum();
  }
  
  return d >= EPS;
#endif
}
