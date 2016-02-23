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

// C/C++ standard library
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// MUSCOD dependencies
#include <def_usrmod.hpp>
#include <model.hpp>

// RBDL Model
#include <grl/policies/rbdl_model.h>

// *****************************************************************************
#ifndef COMMON_NMPC_H
#define COMMON_NMPC_H
// *****************************************************************************
namespace CommonNMPC { // BEGIN NAMESPACE CommonNMPC
// *****************************************************************************

// *****************************************************************************
// Variables
// *****************************************************************************

  extern const unsigned int NMOS;   // Number of phases (Model stages)
  extern const unsigned int NP;     // Number of parameters
  extern const unsigned int NRC;    // Number of coupled constraints
  extern const unsigned int NRCE;   // Number of coupled equality constraints

  extern const unsigned int NXD;    // Number of differential states
  extern const unsigned int NXA;    // Number of algebraic states
  extern const unsigned int NU;     // Number of controls
  extern const unsigned int NPR;    // Number of local parameters

  // dimension constants
  extern const unsigned int LSQFCN_TRACKING_NE;

  extern const unsigned int RDFCN_S_N, RDFCN_S_NE;
  extern const unsigned int RDFCN_E_N, RDFCN_E_NE;

  extern char problem_name[];
  extern std::string rel_data_path;
  extern std::string model_file_name; // any reason for it being const?

  extern RBDLModel rbdl_model;
  extern std::vector<RBDLModel> rbdl_models;

// *****************************************************************************
// Utilities
// *****************************************************************************

  // Hack to allow external programs to alter relative path of DAT files
  extern "C" {
    extern void set_path(std::string new_problem_path, std::string new_lua_model);

    extern bool get_plot_counter();

    extern void set_plot_counter(bool counter);

    extern void convert_obs_for_muscod(const double *from, double *to);
  }

  // Convenience function to check consistency of variable dimensions
  void check_dimensions(
    const unsigned int n_cnt,  const unsigned int n,
    const unsigned int ne_cnt, const unsigned int ne,
    const std::string func_name = "" );

  // load data from file to placeholder
  void load_from_file(
    std::vector<double>& time, std::vector<std::vector<double> >& values,
    const std::string path, const std::string rel_path = ""
  );

  // Get closest value to a reference from vector
  void get_closest(std::vector<double>& vec, const double& value, int& index);

  // Return reference trajectory of states for NMPC
  double ref_xd(const unsigned int choice, const double t);

  // Return reference trajectory of controls for NMPC
  double ref_u(const unsigned int choice, const double t);

// *****************************************************************************
// Objective Functions (Lagrange Type)
// *****************************************************************************

  void lfcn_energy(
    double *ts, double *sd, double *sa, double *u, double *p, double *pr,
    double *res, long *dpnd, InfoPtr *info);

// *****************************************************************************
// Objective Functions (Meyer Type)
// *****************************************************************************

  void mfcn_end_time(
    double *ts, double *xd, double *xa, double *p, double *pr, double *mval,
    long *dpnd, InfoPtr *info);

// *****************************************************************************
// Objective Functions (Least-Squares Type)
// *****************************************************************************

  void lsqfcn_tracking(
    double *ts, double *sd, double *sa, double *u, double *p, double *pr,
    double *res, long *dpnd, InfoPtr *info);

  void lsqfcn_tracking_ref(
    double *ts, double *sd, double *sa, double *u, double *p, double *pr,
    double *res, long *dpnd, InfoPtr *info);

  void msqfcn_tracking(
    double *ts, double *sd, double *sa, double *u, double *p, double *pr,
    double *res, long *dpnd, InfoPtr *info);

// *****************************************************************************
// Right hand sides
// *****************************************************************************

  void ffcn_st (
    double *t, double *xd, double *xa, double *u, double *p, double *rhs,
    double *rwh, long *iwh, InfoPtr *info );

  void ffcn (
    double *t, double *xd, double *xa, double *u, double *p, double *rhs,
    double *rwh, long *iwh, InfoPtr *info );

// *****************************************************************************
// Decoupled Constraints
// *****************************************************************************

  void rdfcn_s (
    double *ts, double *sd, double *sa, double *u, double *p, double *pr,
    double *res, long *dpnd, InfoPtr *info );

  //  Constraints at end point
  void rdfcn_e (
    double *ts, double *sd, double *sa, double *u, double *p, double *pr,
    double *res, long *dpnd, InfoPtr *info );

// *****************************************************************************
// Data Output
// *****************************************************************************

  void data_in(
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
  );

  void data_out(
    double *t, double *sd, double *sa, double *u, double *p,
    double *rwh, long *iwh, InfoPtr *info
  );

  void meshup_output (
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
);

// *****************************************************************************
} // END NAMESPACE CommonNMPC
// *****************************************************************************
// #endif COMMON_NMPC_H
#endif
// *****************************************************************************

