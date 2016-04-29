// *****************************************************************************
// MUSCOD-II example
//
// Implementation of pendulum on a cart using RBDL
//
// Copyright (C) 2015, Manuel Kudruss, IWR, Heidelberg
// *****************************************************************************

// *****************************************************************************
// Includes
// *****************************************************************************

// Includes common code
#include "common_nmpc.h"  // providing functions all NMPC instances share

// *****************************************************************************
// Pre-Processor Macros
// *****************************************************************************

// preprocessor macro allowing use of __func__ macro
#if __STDC_VERSION__ < 199901L
# if __GNUC__ >= 2
#  define __func__ __FUNCTION__
# else
#  define __func__ ""
# endif
#endif

// *****************************************************************************
// Used Namespaces
// *****************************************************************************

// using namespace CommonCode;

// *****************************************************************************
namespace CommonCode { // BEGIN NAMESPACE CommonCode
// *****************************************************************************

// *****************************************************************************
// Model
// *****************************************************************************

// *****************************************************************************
// Constants
// *****************************************************************************


// *****************************************************************************
// Variables
// *****************************************************************************

// *****************************************************************************
// Utilities
// *****************************************************************************

// *****************************************************************************
// Objective Functions (Lagrange Type)
// *****************************************************************************

// ****************************************************************************
// Objectives Mayer Type
// ****************************************************************************

// *****************************************************************************
// Least Squares Functions
// *****************************************************************************

const unsigned int LSQFCN_TRACK_VREF_NREG_NE = 1;
void lsqfcn_track_vref_nreg (
  double *ts, double *sd, double *sa, double *u, double *p, double *pr,
  double *res, long *dpnd, InfoPtr *info
) {
  // define dependencies
  // NOTE: Dependency pattern determines the derivatives to be computed!
  if (*dpnd) {
    // choose dependency pattern
    bool ts_dpnd = false;
    bool sd_dpnd = true;
    bool sa_dpnd = false;
    bool  u_dpnd = false;
    bool  p_dpnd = true;
    bool pr_dpnd = false;
    // automatically resolve dependencies
    *dpnd = RFCN_DPND(
      double(ts_dpnd?*ts:0), double(sd_dpnd?*sd:0),
      double(sa_dpnd?*sa:0), double(u_dpnd?*u:0),
      double(p_dpnd? *p:0), double(pr_dpnd?*pr:0)
    );
    return;
  }
  // define constraint counters
  unsigned int res_ne_cnt = 0;

  // rename for convenience
  const double slope = p[0];
  const double v_ref = p[1];

  const double phi_st = sd[0];
  const double phi_h = sd[1];
  const double dphi_st = sd[2];
  const double dphi_h = sd[3];

  // define constraint residuals here, i.e. res[:_ne] = 0 and res[_ne:] >= 0!
  // NOTE: In C++ the '++' operator as postfix first evaluates the variable
  //       and then increments it, i.e.
  //       n = 0;    -> n  = 0
  //       n2 = n++; -> n2 = 0, n = 1

  // L   = ||r(x)||_2^2
  // r(t,x,u,p) = sqrt(w) * quantity
  // NOTE: tracking of reference velocity
  const double hip_vx = get_hip_vx(phi_st, dphi_st);
  res[res_ne_cnt++] = sw_vref * (v_ref - hip_vx);

  // check dimensions of function
  check_dimensions(0, 0, res_ne_cnt, LSQFCN_TRACK_VREF_NREG_NE, __func__);
}

// -----------------------------------------------------------------------------

const unsigned int MSQFCN_TRACK_AVG_NREG_NE = 1;
void msqfcn_track_avg_nreg(
  double *ts, double *sd, double *sa, double *u, double *p, double *pr, double *res,
  long *dpnd, InfoPtr *info
) {
  if (*dpnd) {
    // choose dependency pattern
    bool ts_dpnd = true;
    bool sd_dpnd = true;
    bool sa_dpnd = false;
    bool  u_dpnd = false;
    bool  p_dpnd = true;
    bool pr_dpnd = false;
    // automatically resolve dependencies
    *dpnd = RFCN_DPND(
      double(ts_dpnd?*ts:0), double(sd_dpnd?*sd:0),
      double(sa_dpnd?*sa:0), double(u_dpnd?*u:0),
      double(p_dpnd? *p:0), double(pr_dpnd?*pr:0)
    );
    return;
  }
  // define constraint counters
  unsigned int res_ne_cnt = 0;

      // rename for convenience
  const double slope = p[0];
  const double v_ref = p[1];

  const double phi_st = sd[0];
  const double phi_h = sd[1];
  const double dphi_st = sd[2];
  const double dphi_h = sd[3];
  const double v_hx = sd[4];

  // define objective
  // define constraint residuals here, i.e. res[:_ne] = 0 and res[_ne:] >= 0!
  // NOTE: In C++ the '++' operator as postfix first evaluates the variable
  //       and then increments it, i.e.
  //       n = 0;    -> n  = 0
  //       n2 = n++; -> n2 = 0, n = 1

  // L   = ||r(x)||_2^2
  // r(t,x,u,p) = sqrt(w) * quantity
  // compute average velocity via
  res[res_ne_cnt++] = sw_vref * ( v_ref - (v_hx / (*ts)) );

  // check dimensions of function
  check_dimensions(0, 0, res_ne_cnt, MSQFCN_TRACK_AVG_NREG_NE, __func__);
}


// *****************************************************************************
// Right Hand Sides
// *****************************************************************************

// *****************************************************************************
// Decoupled Constraints
// *****************************************************************************

// *****************************************************************************
// Data Output
// *****************************************************************************

// *****************************************************************************
} // END NAMESPACE CommonCode
// *****************************************************************************
