// *****************************************************************************
// MUSCOD-II example
//
// Implementation of simplest walker model.
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
#include "def_usrmod.hpp"
#include "model.hpp"

// providing functions all instances share
#include "common_code.h"

// *****************************************************************************
#ifndef COMMON_NMPC_H
#define COMMON_NMPC_H
// *****************************************************************************
namespace CommonCode { // BEGIN NAMESPACE CommonCode
// *****************************************************************************

// *****************************************************************************
// Constants
// *****************************************************************************

// *****************************************************************************
// Variables
// *****************************************************************************

  extern const unsigned int NMOS; // Number of phases (Model stages)
  extern const unsigned int NXD;  // Number of differential states
  extern const unsigned int NXA;  // Number of algebraic states
  extern const unsigned int NU;   // Number of controls
  extern const unsigned int NP;   // Number of parameters
  extern const unsigned int NPR;  // Number of local parameters

  // Number of switching functions
  extern const unsigned int NSWT;

  extern const unsigned int LSQFCN_MIN_TAU;

  extern const unsigned int LSQFCN_TRACK_VREF_WREG_NE;
  extern const unsigned int LSQFCN_TRACK_VREF_NREG_NE;

  extern const unsigned int MSQFCN_TRACK_AVG_WREG_NE;
  extern const unsigned int MSQFCN_TRACK_AVG_NREG_NE;

// *****************************************************************************
// External Scope
// *****************************************************************************

// *****************************************************************************
// Utilities
// *****************************************************************************

// *****************************************************************************
// Objective Functions (Lagrange Type)
// *****************************************************************************

// *****************************************************************************
// Objective Functions (Meyer Type)
// *****************************************************************************

// *****************************************************************************
// Objective Functions (Least-Squares Type)
// *****************************************************************************

  void lsqfcn_min_tau (
    double *ts, double *sd, double *sa, double *u, double *p, double *pr,
    double *res, long *dpnd, InfoPtr *info);

  void lsqfcn_track_vref_wreg(
    double *ts, double *sd, double *sa, double *u, double *p, double *pr,
    double *res, long *dpnd, InfoPtr *info);

  void lsqfcn_track_vref_nreg(
    double *ts, double *sd, double *sa, double *u, double *p, double *pr,
    double *res, long *dpnd, InfoPtr *info);

  void msqfcn_track_avg_wreg(
    double *ts, double *sd, double *sa, double *u, double *p, double *pr,
    double *res, long *dpnd, InfoPtr *info);

  void msqfcn_track_avg_nreg(
    double *ts, double *sd, double *sa, double *u, double *p, double *pr,
    double *res, long *dpnd, InfoPtr *info);

// *****************************************************************************
// Right hand sides
// *****************************************************************************

// *****************************************************************************
// Decoupled Constraints
// *****************************************************************************

// *****************************************************************************
} // END NAMESPACE CommonCode
// *****************************************************************************
// #endif COMMON_NMPC_H
#endif
// *****************************************************************************

