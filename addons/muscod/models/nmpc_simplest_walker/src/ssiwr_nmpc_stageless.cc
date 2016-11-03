// *****************************************************************************
// MUSCOD-II example
//
// Implementation of simplest walker model.
//
// Copyright (C) 2015, Manuel Kudruss, IWR, Heidelberg
// *****************************************************************************

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "def_usrmod.hpp"
#include "model.hpp"

// *****************************************************************************
// Constants
// *****************************************************************************

// define MUSCOD-II Dimensions
static const unsigned int NMOS = 1;  /* Number of phases (MOdel Stages) */
static const unsigned int NXD  = 4;  /* Number of differential states */
static const unsigned int NXA  = 0;  /* Number of algebraic states */
static const unsigned int NU   = 1;  /* Number of controls */
static const unsigned int NP   = 2;  /* Number of parameters */
static const unsigned int NPR  = 0;  /* Number of local parameters */

// tolerance of parallelism
static const double parallel_TOL = 0.15;

// define square roots of weights for LSQ objective
// RL discount factor
static const double rl_y = 1.000;
static const double rl_sy = sqrt(rl_y);

// weights
static const double w_vref   =   10.00;
static const double w_tau    =   00.01;
static const double w_ell_fh = 1000.00;
static const double w_periodicity = 1000.0;

// define square roots of weights for LSQ objective
static const double sw_vref = sqrt(w_vref);
static const double sw_tau = sqrt(w_tau);
static const double sw_ell_fh = sqrt(w_ell_fh);
static const double sw_periodicity = sqrt(w_periodicity);

// *****************************************************************************
// Variables
// *****************************************************************************

static bool is_problem_name_initialized = false;
static bool add_counter = false;
static char problem_name[255];
static long real_nmos, real_nmsn;

static std::string rel_data_path = "";

// placeholder for plotting
static std::vector<double> _plotting_t_values;
static std::vector<double> _plotting_p_values;
static std::vector<std::vector<double> > _plotting_sd_values;
static std::vector<std::vector<double> > _plotting_u_values;

// *****************************************************************************
// External Scope
// *****************************************************************************

extern "C" {
  void set_path(const std::string new_path){
    rel_data_path = new_path;
    std::cout << "NMPC problem, setting new data path to: '";
    std::cout << rel_data_path << "'" <<std::endl;
  }

  bool get_plot_counter(){
    return add_counter;
  }

  void set_plot_counter(bool counter){
    add_counter = counter;
    std::cout << "plot counter is set to: " << get_plot_counter() << std::endl;
  }
} // END of extern "C"

// *****************************************************************************
// Utilities
// *****************************************************************************

// Convenience function to check consistency of variable dimensions
static void check_dimensions(
    const unsigned int n_cnt,  const unsigned int n,
    const unsigned int ne_cnt, const unsigned int ne,
    const std::string func_name
){
  if ((n_cnt != n) || (ne_cnt != ne)) {
    if (func_name != "") {
      std::cout << "Dimensions of function '" << func_name;
      std::cout << "' are inconsistent!" << std::endl;
    }
    std::cout << "n_cnt:  " << n_cnt  << " (" << n  << ")" << std::endl;
    std::cout << "ne_cnt: " << ne_cnt << " (" << ne << ")" << std::endl;
    std::cout << std::endl;
    std::cout << "bailing out..." << std::endl;
    abort();
  }
  return;
}

// convenience functions from MPRL implementation
static double get_hip_px(const double& phi_st) {
  return -sin(phi_st);
}

static double get_hip_py(const double& phi_st) {
  return cos(phi_st);
}

static double get_swing_foot_px(const double& phi_st, const double& phi_h) {
  return get_hip_px(phi_st) + sin(phi_st - phi_h);
}

static double get_swing_foot_py(const double& phi_st, const double& phi_h) {
  return get_hip_py(phi_st) - cos(phi_st - phi_h);
}

static double get_hip_vx(const double& phi_st, const double& dphi_st) {
  return -cos(phi_st) * dphi_st;
}

static double get_hip_vy(const double& phi_st, const double& dphi_st) {
  return -sin(phi_st) * dphi_st;
}

static double get_swing_foot_vx(
  const double& phi_st, const double& phi_h,
  const double& dphi_st, const double& dphi_h
) {
  return get_hip_vx(phi_st, dphi_st) + cos(phi_st - phi_h) * (dphi_st - dphi_h);
}

static double get_swing_foot_vy(
  const double& phi_st, const double& phi_h,
  const double& dphi_st, const double& dphi_h
) {
  return get_hip_vy(phi_st, dphi_st) + sin(phi_st - phi_h) * (dphi_st - dphi_h);
}


// *****************************************************************************
// Objective Funtions
// *****************************************************************************

// *****************************************************************************
// Least Squares Functions
// *****************************************************************************

static const unsigned int LSQFCN_TRACK_VREF_WREG_NE = 2;
static void lsqfcn_track_vref_wreg (
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
    bool  u_dpnd = true;
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

  const double tau = u[0];

  // define constraint residuals here, i.e. res[:_ne] = 0 and res[_ne:] >= 0!
  // NOTE: In C++ the '++' operator as postfix first evaluates the variable
  //       and then increments it, i.e.
  //       n = 0;    -> n  = 0
  //       n2 = n++; -> n2 = 0, n = 1

  // L   = ||r(x)||_2^2
  // r(t,x,u,p) = sqrt(w) * quantity
  // NOTE: add RL discount to trajectory
  double discount = pow(rl_y, double(info->cnode)/2.);

  // NOTE: tracking of reference velocity
  const double hip_vx = get_hip_vx(phi_st, dphi_st);
  res[res_ne_cnt++] = sw_vref * (v_ref - hip_vx) * discount;
  res[res_ne_cnt++] = sw_tau * tau * discount;

  // check dimensions of function
  check_dimensions(0, 0, res_ne_cnt, LSQFCN_TRACK_VREF_WREG_NE, __func__);
}

// -----------------------------------------------------------------------------

static const unsigned int LSQFCN_TRACK_VREF_NREG_NE = 1;
static void lsqfcn_track_vref_nreg (
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
  // NOTE: add RL discount to trajectory
  double discount = pow(rl_y, double(info->cnode)/2.);

  // NOTE: tracking of reference velocity
  const double hip_vx = get_hip_vx(phi_st, dphi_st);
  res[res_ne_cnt++] = sw_vref * (v_ref - hip_vx) * discount;

  // check dimensions of function
  check_dimensions(0, 0, res_ne_cnt, LSQFCN_TRACK_VREF_NREG_NE, __func__);
}

// -----------------------------------------------------------------------------

static const unsigned int LSQFCN_MIN_TAU = 1;
static void lsqfcn_min_tau(
  double *ts, double *sd, double *sa, double *u, double *p, double *pr,
  double *res, long *dpnd, InfoPtr *info
) {
  // define dependencies
  // NOTE: Dependency pattern determines the derivatives to be computed!
  if (*dpnd) {
    // choose dependency pattern
    bool ts_dpnd = false;
    bool sd_dpnd = false;
    bool sa_dpnd = false;
    bool  u_dpnd = true;
    bool  p_dpnd = false;
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

  const double tau = u[0];

  // define constraint residuals here, i.e. res[:_ne] = 0 and res[_ne:] >= 0!
  // NOTE: In C++ the '++' operator as postfix first evaluates the variable
  //       and then increments it, i.e.
  //       n = 0;    -> n  = 0
  //       n2 = n++; -> n2 = 0, n = 1

  // L   = ||r(x)||_2^2
  // r(t,x,u,p) = sqrt(w) * quantity
  // NOTE: add RL discount to trajectory
  double discount = pow(rl_y, double(info->cnode)/2.);

  // NOTE: tracking of reference velocity
  res[res_ne_cnt++] = sw_tau * tau * discount;

  // check dimensions of function
  check_dimensions(0, 0, res_ne_cnt, LSQFCN_MIN_TAU, __func__);
}

// -----------------------------------------------------------------------------

static const unsigned int LSQFCN_MIN_TAU_AND_PERIODICTY = 1+4;
static void lsqfcn_min_tau_and_periodicity(
  double *ts, double *sd, double *sa, double *u, double *p, double *pr,
  double *res, long *dpnd, InfoPtr *info
) {
  // define dependencies
  // NOTE: Dependency pattern determines the derivatives to be computed!
  if (*dpnd) {
    // choose dependency pattern
    bool ts_dpnd = false;
    bool sd_dpnd = false;
    bool sa_dpnd = false;
    bool  u_dpnd = true;
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

  const double slack0 = p[2];
  const double slack1 = p[3];
  const double slack2 = p[4];
  const double slack3 = p[5];

  const double phi_st = sd[0];
  const double phi_h = sd[1];
  const double dphi_st = sd[2];
  const double dphi_h = sd[3];

  const double tau = u[0];

  // define constraint residuals here, i.e. res[:_ne] = 0 and res[_ne:] >= 0!
  // NOTE: In C++ the '++' operator as postfix first evaluates the variable
  //       and then increments it, i.e.
  //       n = 0;    -> n  = 0
  //       n2 = n++; -> n2 = 0, n = 1

  // L   = ||r(x)||_2^2
  // r(t,x,u,p) = sqrt(w) * quantity
  // NOTE: add RL discount to trajectory
  double discount = pow(rl_y, double(info->cnode)/2.);

  // NOTE: tracking of reference velocity
  res[res_ne_cnt++] = sw_tau * tau * discount;

  res[res_ne_cnt++] = sw_periodicity * slack0;
  res[res_ne_cnt++] = sw_periodicity * slack1;
  res[res_ne_cnt++] = sw_periodicity * slack2;
  res[res_ne_cnt++] = sw_periodicity * slack3;

  // check dimensions of function
  check_dimensions(0, 0, res_ne_cnt, LSQFCN_MIN_TAU_AND_PERIODICTY, __func__);
}

// *****************************************************************************
// Right Hand Sides
// *****************************************************************************

// Right-hand side for the single support phase
static void ffcn (
  double *t, double *xd, double *xa, double *u, double *p, double *rhs,
  double *rwh, long *iwh, InfoPtr *info
) {
  // define rhs counter
  unsigned int rhs_cnt = 0;

  // rename for convenience
  const double sigma   =  p[0];
  const double act     =  u[0];

  const double phi_st  = xd[0];
  const double phi_h   = xd[1];
  const double dphi_st = xd[2];
  const double dphi_h  = xd[3];

  // define right-hand side
  rhs[rhs_cnt++] = dphi_st;
  rhs[rhs_cnt++] = dphi_h;
  rhs[rhs_cnt++] = sin(phi_st - sigma);
  rhs[rhs_cnt++] = sin(phi_h)*(dphi_st*dphi_st - cos(phi_st - sigma))
                 + sin(phi_st - sigma) + act;

  // check dimensions of functions
  check_dimensions(0, 0, rhs_cnt, NXD, __func__);
}

// Right-hand side for the transition phase with augmented states
static void ffcn_trans (
  double *t, double *xd, double *xa, double *u, double *p, double *rhs,
  double *rwh, long *iwh, InfoPtr *info
) {
  // define rhs counter
  unsigned int rhs_cnt = 0;

  // rename for convenience
  double sigma   =  p[0];
  double act     =  u[0];

  double phi_st  = xd[0];
  double phi_h   = xd[1];
  double dphi_st = xd[2];
  double dphi_h  = xd[3];

  // [      \phi_st+ ] = [ -1 0 0                                   0 ] [      \phi_st- ]
  // [      \phi_h+  ] = [ -2 0 0                                   0 ] [      \phi_h-  ]
  // [ \dot \phi_st+ ] = [  0 0 cos(2*\phi_st-)                     0 ] [ \dot \phi_st- ]
  // [ \dot \phi_h+  ] = [  0 0 cos(2*\phi_st-)*(1-cos(2*\phi_st-)) 0 ] [ \dot \phi_h-  ]
  rhs[rhs_cnt++] = -1.0*phi_st;
  rhs[rhs_cnt++] = -2.0*phi_st;
  rhs[rhs_cnt++] = dphi_st*cos(2.0*phi_st);
  rhs[rhs_cnt++] = dphi_st*cos(2.0*phi_st)*(1.0 - cos(2*phi_st));

  // check dimensions of functions
  check_dimensions(0, 0, rhs_cnt, NXD, __func__);
}


static const unsigned int NSWT = 1; // Number of switching functions

static void detect_switch_fcn (
  double *t, double *xd, double *xa, double* u, double *p,
  long *DUMMY, long *iswt, long *nswt, double *res,
  double *rwh, long *iwh, InfoPtr *info
) {
  // rename for convenience
  const double phi_st = xd[0];
  const double phi_h = xd[1];

  // check for ground contact of swing foot
  res[0] = get_swing_foot_py(phi_st, phi_h);
}

// *****************************************************************************

static void execute_switch_fcn (
  double *t, double *xd, double *xa, double *u, double *p,
  long iswt, double *rwh, long *iwh, InfoPtr *info
) {
  // define rhs counter
  static unsigned int rhs_cnt = 0;

  // rename for convenience
  static double rhs[4] = {0., 0., 0., 0.};

  const double sigma   =  p[0];
  const double act     =  u[0];

  const double phi_st  = xd[0];
  const double phi_h   = xd[1];
  const double dphi_st = xd[2];
  const double dphi_h  = xd[3];

  const bool debug_output = false;
  if (debug_output) {
    std::cout << "switch detected at t = " << *t << std::endl;
    std::cout << "switch signature: info->swt[" << iswt << "] = " << info->swt[iswt] << std::endl;
  }

  // confirm switch
  if ((get_swing_foot_vy(phi_st, phi_h, dphi_st, dphi_h) < 0)) // Penetrating
  {
    if (debug_output) {
      std::cout << "penetrating contact: swf_vx =" << get_swing_foot_vy(phi_st, phi_h, dphi_st, dphi_h) << " < 0" << std::endl;
    }
    if((phi_h*phi_h >= parallel_TOL*parallel_TOL)) // forward step
    {
      if (debug_output) {
        std::cout << "feet not parallel: " << sqrt(phi_h*phi_h) << " = |phi_h| >= TOL = " << parallel_TOL << std::endl;
        std::cout << "SWITCH CONFIRMED!" << std::endl;
        std::cout << std::endl;
      }
      // [      \phi_st+ ] = [ -1 0 0                                   0 ] [      \phi_st- ]
      // [      \phi_h+  ] = [ -2 0 0                                   0 ] [      \phi_h-  ]
      // [ \dot \phi_st+ ] = [  0 0 cos(2*\phi_st-)                     0 ] [ \dot \phi_st- ]
      // [ \dot \phi_h+  ] = [  0 0 cos(2*\phi_st-)*(1-cos(2*\phi_st-)) 0 ] [ \dot \phi_h-  ]
      rhs[0] = -1.0*phi_st;
      rhs[1] = -2.0*phi_st;
      rhs[2] = dphi_st*cos(2.0*phi_st);
      rhs[3] = dphi_st*cos(2.0*phi_st)*(1.0 - cos(2*phi_st));

      // apply transition function
      xd[0] = rhs[0];
      xd[1] = rhs[1];
      xd[2] = rhs[2];
      xd[3] = rhs[3];

      // leave scope of function
      return;
    }
  }
  // else proceed with nominal solution
  if (debug_output) {
    std::cout << "SWITCH REJECTED!" << std::endl;
    std::cout << std::endl;
  }
  // define right-hand side
  xd[0] = phi_st;
  xd[1] = phi_h;
  xd[2] = dphi_st;
  xd[3] = dphi_h;
}

static void execute_switch_fcn_avg (
  double *t, double *xd, double *xa, double *u, double *p,
  long iswt, double *rwh, long *iwh, InfoPtr *info
) {
  // define rhs counter
  static unsigned int rhs_cnt = 0;

  // rename for convenience
  static double rhs[5] = {0., 0., 0., 0., 0.};

  const double sigma   =  p[0];
  const double act     =  u[0];

  const double phi_st  = xd[0];
  const double phi_h   = xd[1];
  const double dphi_st = xd[2];
  const double dphi_h  = xd[3];
  const double v_hx    = xd[4];

  const bool debug_output = false;
  if (debug_output) {
    std::cout << "switch detected at t = " << *t << std::endl;
    std::cout << "switch signature: info->swt[" << iswt << "] = " << info->swt[iswt] << std::endl;
  }

  // confirm switch
  if ((get_swing_foot_vy(phi_st, phi_h, dphi_st, dphi_h) < 0)) // Penetrating
  {
    if (debug_output) {
      std::cout << "penetrating contact: swf_vx =" << get_swing_foot_vy(phi_st, phi_h, dphi_st, dphi_h) << " < 0" << std::endl;
    }
    if((phi_h*phi_h >= parallel_TOL*parallel_TOL)) // forward step
    {
      if (debug_output) {
        std::cout << "feet not parallel: " << sqrt(phi_h*phi_h) << " = |phi_h| >= TOL = " << parallel_TOL << std::endl;
        std::cout << "SWITCH CONFIRMED!" << std::endl;
        std::cout << std::endl;
      }
      // [      \phi_st+ ] = [ -1 0 0                                   0 ] [      \phi_st- ]
      // [      \phi_h+  ] = [ -2 0 0                                   0 ] [      \phi_h-  ]
      // [ \dot \phi_st+ ] = [  0 0 cos(2*\phi_st-)                     0 ] [ \dot \phi_st- ]
      // [ \dot \phi_h+  ] = [  0 0 cos(2*\phi_st-)*(1-cos(2*\phi_st-)) 0 ] [ \dot \phi_h-  ]
      rhs[0] = -1.0*phi_st;
      rhs[1] = -2.0*phi_st;
      rhs[2] = dphi_st*cos(2.0*phi_st);
      rhs[3] = dphi_st*cos(2.0*phi_st)*(1.0 - cos(2*phi_st));
      rhs[4] = v_hx;

      // apply transition function
      xd[0] = rhs[0];
      xd[1] = rhs[1];
      xd[2] = rhs[2];
      xd[3] = rhs[3];
      xd[4] = rhs[4];

      // leave scope of function
      return;
    }
  }
  // else proceed with nominal solution
  if (debug_output) {
    std::cout << "SWITCH REJECTED!" << std::endl;
    std::cout << std::endl;
  }
  // define right-hand side
  xd[0] = phi_st;
  xd[1] = phi_h;
  xd[2] = dphi_st;
  xd[3] = dphi_h;
  xd[4] = v_hx;
}

// *****************************************************************************
// Decoupled Constraints
// *****************************************************************************

//                 # of all constraints # of equality constraints
static const unsigned int RDFCN_TRANS_N = 4,   RDFCN_TRANS_NE = 1;
static void rdfcn_trans(
  double *ts, double *sd, double *sa, double *u, double *p, double *pr, double *res,
  long *dpnd, InfoPtr *info
) {
  // define dependencies
  // NOTE: Dependency pattern determines the derivatives to be computed!
  if (*dpnd) {
    // choose dependency pattern
    bool ts_dpnd = false;
    bool sd_dpnd = true;
    bool sa_dpnd = false;
    bool  u_dpnd = false;
    bool  p_dpnd = false;
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
  unsigned int res_n_cnt  = 0;
  unsigned int res_ne_cnt = 0;

  // rename for convenience
  double sigma   =  p[0];
  double act     =  u[0];

  double phi_st  = sd[0];
  double phi_h   = sd[1];
  double dphi_st = sd[2];
  double dphi_h  = sd[3];

  // define constraint residuals here, i.e. res[:_ne] = 0 and res[_ne:] >= 0!
  // NOTE: In C++ the '++' operator as postfix first evaluates the variable
  //       and then increments it, i.e.
  //       n = 0;    -> n  = 0
  //       n2 = n++; -> n2 = 0, n = 1

  // Swing foot passed through the floor
  res[res_n_cnt++] = get_swing_foot_py(phi_st, phi_h); res_ne_cnt++; // = 0
  res[res_n_cnt++] = -get_swing_foot_vy(phi_st, phi_h, dphi_st, dphi_h); // >= 0

  // forward step in direction of movement
  //res[res_n_cnt++] = getSwingFootX(phi_st, phi_h);
  res[res_n_cnt++] = -1.0 * dphi_st; // >= 0
  res[res_n_cnt++] = phi_h*phi_h - 1e-2; // >= 0

  // check dimensions of constraints
  check_dimensions(
    res_n_cnt, RDFCN_TRANS_N, res_ne_cnt, RDFCN_TRANS_NE, __func__
  );
}


///  Constraints ensuring not to fall or penetrate ground
//                 # of all constraints     # of equality constraints
static const unsigned int RDFCN_FEASIBILITY_N = 4, RDFCN_FEASIBILITY_NE = 0;
static void rdfcn_feasibility(
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
    bool  p_dpnd = false;
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
  unsigned int res_n_cnt  = 0;
  unsigned int res_ne_cnt = 0;

  // rename for convenience
  // const double sigma   =  p[0];
  // const double act     =  u[0];

  const double phi_st  = sd[0];
  const double phi_h   = sd[1];
  const double dphi_st = sd[2];
  const double dphi_h  = sd[3];

  // define constraint residuals here, i.e. res[:_ne] = 0 and res[_ne:] >= 0!
  // NOTE: In C++ the '++' operator as postfix first evaluates the variable
  //       and then increments it, i.e.
  //       n = 0;    -> n  = 0
  //       n2 = n++; -> n2 = 0, n = 1

  res[res_n_cnt++] = get_hip_py(phi_st); // >= 0.0
  res[res_n_cnt++] = get_swing_foot_py(phi_st, phi_h) - (-0.000); // >= 0.0
  // res[res_n_cnt++] = get_hip_vx(phi_st, dphi_st); // >= 0.0
  // res[res_n_cnt++] = get_swing_foot_vx(phi_st, phi_h, dphi_st, dphi_h); // >= 0.0
  // |phi_st| > Pi/8 = 0.3926875
  res[res_n_cnt++] = 0.3926875*0.3926875 - phi_st*phi_st; // >= 0.0
  // |phi_h - 2*phi_st| > Pi/4 = 0.78
  res[res_n_cnt++] = 0.78*0.78 - (phi_h - 2*phi_st)*(phi_h - 2*phi_st); // >= 0.0

  // check dimensions of constraints
  check_dimensions(
    res_n_cnt, RDFCN_FEASIBILITY_N, res_ne_cnt, RDFCN_FEASIBILITY_NE, __func__
  );
}

// *****************************************************************************
// Coupled Constraints
// *****************************************************************************

static const unsigned int RCFCN_N = 4, RCFCN_NE = 4;
static void rcfcn_s (
  double *ts, double *sd, double *sa, double *u, double *p, double *pr, double *res,
  long *dpnd, InfoPtr *info
) {
  // define dependencies
  // NOTE: Dependency pattern determines the derivatives to be computed!
  if (*dpnd) {
    // choose dependency pattern
    bool ts_dpnd = false;
    bool sd_dpnd = true;
    bool sa_dpnd = false;
    bool  u_dpnd = false;
    bool  p_dpnd = false;
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
  unsigned int res_n_cnt  = 0;
  unsigned int res_ne_cnt = 0;

  // rename for convenience
  double phi_st  = sd[0];
  double phi_h   = sd[1];
  double dphi_st = sd[2];
  double dphi_h  = sd[3];

  // define constraint residuals here, i.e. res[:_ne] = 0 and res[_ne:] >= 0!
  // NOTE: In C++ the '++' operator as postfix first evaluates the variable
  //       and then increments it, i.e.
  //       n = 0;    -> n  = 0
  //       n2 = n++; -> n2 = 0, n = 1

  res[res_n_cnt++] = 1.0 * phi_st;  res_ne_cnt++;
  res[res_n_cnt++] = 1.0 * phi_h;   res_ne_cnt++;
  res[res_n_cnt++] = 1.0 * dphi_st; res_ne_cnt++;
  res[res_n_cnt++] = 1.0 * dphi_h;  res_ne_cnt++;

  // check dimensions of constraints
  check_dimensions(
    res_n_cnt, RCFCN_N, res_ne_cnt, RCFCN_NE, __func__
  );
}

static void rcfcn_e (
  double *ts, double *sd, double *sa, double *u, double *p, double *pr, double *res,
  long *dpnd, InfoPtr *info
) {
  // define dependencies
  // NOTE: Dependency pattern determines the derivatives to be computed!
  if (*dpnd) {
    // choose dependency pattern
    bool ts_dpnd = false;
    bool sd_dpnd = true;
    bool sa_dpnd = false;
    bool  u_dpnd = false;
    bool  p_dpnd = false;
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
  unsigned int res_n_cnt  = 0;
  unsigned int res_ne_cnt = 0;

  // rename for convenience
  double phi_st  = sd[0];
  double phi_h   = sd[1];
  double dphi_st = sd[2];
  double dphi_h  = sd[3];

  // define constraint residuals here, i.e. res[:_ne] = 0 and res[_ne:] >= 0!
  // NOTE: In C++ the '++' operator as postfix first evaluates the variable
  //       and then increments it, i.e.
  //       n = 0;    -> n  = 0
  //       n2 = n++; -> n2 = 0, n = 1

  res[res_n_cnt++] = -1.0 * phi_st;  res_ne_cnt++;
  res[res_n_cnt++] = -1.0 * phi_h;   res_ne_cnt++;
  res[res_n_cnt++] = -1.0 * dphi_st; res_ne_cnt++;
  res[res_n_cnt++] = -1.0 * dphi_h;  res_ne_cnt++;

  // check dimensions of constraints
  check_dimensions(
    res_n_cnt, RCFCN_N, res_ne_cnt, RCFCN_NE, __func__
  );
}

// *****************************************************************************
// Data Input
// *****************************************************************************

static void data_in (
  long   *imos,      /* index of model stage (I) */
  long   *imsn,      /* index of m.s. node on current model stage (I) */
  double *sd,        /* differential states at m.s. node (I/O) */
  double *sa,        /* algebraic states at m.s. node (I/O) */
  double *u,         /* controls at m.s. node (I/O) */
  double *udot,      /* control slopes at m.s. node (I/O) */
  double *ue,        /* controls at end of m.s. interval (I/O) */
  double *uedot,     /* control slopes at end of m.s. interval (I/O) */
  double *p,         /* global model parameters (I/O) */
  double *h,         /* model stage durations (I/O) */
  double *pr         /* local i.p.c. parameters (I/O) */
 ) {
  // show configuration
  static bool has_shown_configuration = false;
  if (!has_shown_configuration) {
    std::cout << "MODEL CONFIGURATION" << std::endl;
    std::cout << std::endl;
    std::cout << "NMOS = " << NMOS << std::endl;
    std::cout << "NP   = " << NP   << std::endl;
    // std::cout << "NRC  = " << NRC  << std::endl;
    // std::cout << "NRCE = " << NRCE << std::endl;
    std::cout << std::endl;
    std::cout << "NXD  = " << NXD  << std::endl;
    std::cout << "NXA  = " << NXA  << std::endl;
    std::cout << "NU   = " << NU   << std::endl;
    std::cout << "NPR  = " << NPR  << std::endl;

    has_shown_configuration = true;
  }

  // count model stages and shooting nodes
  real_nmos = std::max(real_nmos, *imos);
  real_nmsn = std::max(real_nmsn, *imsn);
}

// *****************************************************************************
// Data Output
// *****************************************************************************

static void data_out(
  double *t, double *sd, double *sa, double *u, double *p,
  double *rwh, long *iwh, InfoPtr *info
  ) {
  static long lnode = -1;
  static long iteration_cnt = 0;
    // get problem name from MUSCOD
    // NOTE: fixed size array could cause problems for larger problem names
  if (!is_problem_name_initialized) {
    memset (problem_name, 0, 255);
    get_pname (problem_name);
  }

  if (*t == 0.) {
    std::ofstream meshup_csv_stream;

    // crate string with purpose suffix
    std::string meshup_header_file = std::string("RES/meshup_");
    std::string data_sd_file       = std::string("sd_");
    std::string data_u_file        = std::string("u_");
    std::string data_p_file        = std::string("p_");

    // add problem name identifier
    meshup_header_file += std::string(problem_name);
    data_sd_file       += std::string(problem_name);
    data_u_file        += std::string(problem_name);
    data_p_file        += std::string(problem_name);

    // add iteration counter [optional]
    if (add_counter) {
      std::stringstream sstm;
      sstm << std::string("_");
      // NOTE: equivalent to "%03d"
      sstm << std::setfill('0') << std::setw(4) << iteration_cnt;
      meshup_header_file += sstm.str();
      data_sd_file       += sstm.str();
      data_u_file        += sstm.str();
      data_p_file        += sstm.str();
    }

    // add file identifier suffix
    meshup_header_file += std::string(".csv");
    data_sd_file       += std::string(".csv");
    data_u_file        += std::string(".csv");
    data_p_file        += std::string(".csv");

    std::ofstream meshup_header_stream;
    std::ofstream data_sd_stream;
    std::ofstream data_u_stream;
    std::ofstream data_p_stream;

    meshup_header_stream.open (meshup_header_file.c_str(),                   std::ios_base::trunc);
    data_sd_stream.      open ((std::string("RES/") + data_sd_file).c_str(), std::ios_base::trunc);
    data_u_stream.       open ((std::string("RES/") + data_u_file). c_str(), std::ios_base::trunc);
    data_p_stream.       open ((std::string("RES/") + data_p_file). c_str(), std::ios_base::trunc);

    if (
      !meshup_header_stream || !data_sd_stream || !data_u_stream ||
      !data_p_stream
    ) {
      std::cerr << "Error opening file ";
      if (!meshup_header_stream) {
        std::cerr << meshup_header_file;
      }
      if (!data_sd_stream) {
        std::cerr << data_sd_file;
      }
      if (!data_u_stream) {
        std::cerr << data_u_file;
      }
      if (!data_p_stream) {
        std::cerr << data_p_file;
      }
      std::cerr << std::endl;
      abort();
    }
    const char* meshup_header = "";
    meshup_header_stream << meshup_header << std::endl;
    meshup_header_stream << "DATA_FROM: " << data_sd_file << std::endl;

    if (_plotting_t_values.size() > 0) {
      // save time dependent quantities
      for (unsigned int i = 0; i < _plotting_t_values.size(); i ++) {
        // states
        data_sd_stream << _plotting_t_values[i] << ", ";
        for (unsigned int j = 0; j < _plotting_sd_values[i].size(); j++) {
          data_sd_stream << _plotting_sd_values[i][j];
          if (j < _plotting_sd_values[i].size() -1 )
            data_sd_stream << ", ";
        }
        data_sd_stream << std::endl;

        // controls
        data_u_stream << _plotting_t_values[i] << ", ";
        for (unsigned int j = 0; j < _plotting_u_values[i].size(); j++) {
          data_u_stream << _plotting_u_values[i][j];
          if (j < _plotting_u_values[i].size() -1 )
            data_u_stream << ", ";
        }
        data_u_stream << std::endl;
      }

      // save time independent quantities
      // parameters
      for (unsigned int j = 0; j < _plotting_p_values.size(); j++) {
        data_p_stream << _plotting_p_values[j];
        if (j < _plotting_p_values.size() -1 )
          data_p_stream << ", ";
      }
      data_p_stream << std::endl;

      iteration_cnt++;
    }

    _plotting_t_values.clear();
    _plotting_sd_values.clear();
    _plotting_u_values.clear();
    _plotting_p_values.clear();

    meshup_header_stream.close();
    data_sd_stream.close();
    data_u_stream.close();
    data_p_stream.close();

    lnode = -1;
  }

  if (info->cnode != lnode) {
    if (info->cnode == 0) {
      for (unsigned i = 0; i < NP; i++) {
        _plotting_p_values.push_back(p[i]);
      }
    }
    // pass new node value
    lnode = info->cnode;
  }

  _plotting_t_values.push_back (*t);

  std::vector<double> sd_vec (NXD);
  for (unsigned i = 0; i < NXD; i++)
    sd_vec[i] = sd[i];
  _plotting_sd_values.push_back (sd_vec);

  std::vector<double> u_vec (NU);
  for (unsigned i = 0; i < NU; i++)
    u_vec[i] = u[i];
  _plotting_u_values.push_back (u_vec);
}

static void meshup_out
(
    long   *imos,      ///< index of model stage (I)
    long   *imsn,      ///< index of m.s. node on current model stage (I)
    double *ts,        ///< time at m.s. node (I)
    double *te,        ///< time at end of m.s. interval (I)
    double *sd,        ///< differential states at m.s. node (I)
    double *sa,        ///< algebraic states at m.s. node (I)
    double *u,         ///< controls at m.s. node (I)
    double *udot,      ///< control slopes at m.s. node (I)
    double *ue,        ///< controls at end of m.s. interval (I)
    double *uedot,     ///< control slopes at end of m.s. interval (I)
    double *p,         ///< global model parameters (I)
    double *pr,        ///< local i.p.c. parameters (I)
    double *ccxd,
    double *mul_ccxd,  ///< multipliers of continuity conditions (I)
  #if defined(PRSQP) || defined(EXTPRSQP)
    double *ares,
    double *mul_ares,
  #endif
    double *rd,
    double *mul_rd,    ///< multipliers of decoupled i.p.c. (I)
    double *rc,
    double *mul_rc,    ///< multipliers of coupled i.p.c. (I)
    double *obj,
    double *rwh,       ///< real work array (I)
    long   *iwh        ///< integer work array (I)
  ) {
    InfoPtr info(0, *imos, *imsn);
    data_out( ts, sd, sa, u, p, rwh, iwh, &info);
  }

// *****************************************************************************
// MUSCOD Application
// *****************************************************************************

// Entry point for the muscod application
extern "C" void def_model(void);
void def_model(void) {

  // define problem dimensions
  // NOTE: enforce periodicity on motion
  // def_mdims(NMOS, NP, 0, 0);
  def_mdims(NMOS, NP, RCFCN_N, RCFCN_NE);

  // ***************************************************************************
  // Model Stage: Lead in motion
  // ***************************************************************************
  unsigned long imos = 0;

  /*
  def_mstage(
       imos, // imos,
    // nxd, nxa, nu,
       NXD, NXA, NU,
       NULL, // MayPtr mfcn,
       NULL, // LagPtr lfcn,
    // jacmlo, jacmup, astruc,
       0, 0, 0,
    // MatPtr afcn, RHSPtr ffcn, RHSPtr gfcn,
       NULL, ffcn, NULL,
    // rwh,  iwh
       NULL, NULL
  );

  // define least-squares objective
  def_lsq(imos, "*", NPR,
    LSQFCN_TRACK_VREF_WREG_NE, lsqfcn_track_vref_wreg
  );

  // define switching behavior for single stage formulation
  def_swt(imos, NSWT,
    &detect_switch_fcn, &execute_switch_fcn
  );

  // define constraints
  // NOTE: enforce periodicity on motion
  def_mpc(
    imos, "s", NPR,
    RDFCN_FEASIBILITY_N, RDFCN_FEASIBILITY_NE,
    rdfcn_feasibility,
    NULL
    // rcfcn_s
  );

  def_mpc(imos, "i", NPR,
    RDFCN_FEASIBILITY_N, RDFCN_FEASIBILITY_NE,
    rdfcn_feasibility,
    NULL
  );

  // increment model stage
  imos++;
  */

  // ***************************************************************************
  // Model Stage: Periodic motion
  // ***************************************************************************

  def_mstage(
       imos, // imos,
    // nxd, nxa, nu,
       NXD, NXA, NU,
       NULL, // MayPtr mfcn,
       NULL, // LagPtr lfcn,
    // jacmlo, jacmup, astruc,
       0, 0, 0,
    // MatPtr afcn, RHSPtr ffcn, RHSPtr gfcn,
       NULL, ffcn, NULL,
    // rwh,  iwh
       NULL, NULL
  );

  // define least-squares objective
  def_lsq(imos, "*", NPR,
    LSQFCN_TRACK_VREF_WREG_NE, lsqfcn_track_vref_wreg
  );

  // define switching behavior for single stage formulation
  def_swt(imos, NSWT,
    &detect_switch_fcn, &execute_switch_fcn
  );

  // define constraints
  // NOTE: enforce periodicity on motion
  def_mpc(
    imos, "s", NPR,
    RDFCN_FEASIBILITY_N, RDFCN_FEASIBILITY_NE,
    rdfcn_feasibility,
     // NULL
    rcfcn_s
  );

  def_mpc(imos, "i", NPR,
    RDFCN_FEASIBILITY_N, RDFCN_FEASIBILITY_NE,
    rdfcn_feasibility,
    NULL
  );

  // NOTE: enforce periodicity on motion at end of motion
  def_mpc(
    imos, "e", NPR,
    RDFCN_FEASIBILITY_N, RDFCN_FEASIBILITY_NE,
    rdfcn_feasibility,
     // NULL
    rcfcn_e
  );

  // increment model stage
  imos++;

  // check number of model stages
  check_dimensions(0, 0, imos, NMOS, __func__);

  // specify setup, tear-down and plotting functions
  // NOTE: data_in is used to calculate current NMOS and NMS
  //       or instantiate different RBLD models for parallel evaluation of
  //       shooting nodes.
  def_mio (data_in, NULL, NULL);
  // def_mio (data_in , meshup_out, data_out);
}
