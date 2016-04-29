// *****************************************************************************
// MUSCOD-II example
//
// Implementation of simplest walker model.
//
// Copyright (C) 2015, Manuel Kudruss, IWR, Heidelberg
// *****************************************************************************

#include "def_usrmod.hpp"
#include "model.hpp"
#include "common_code.h"
#include "common_nmpc.h"

// *****************************************************************************
// Constants
// *****************************************************************************

// define MUSCOD-II Dimensions
// NOTE: To resolve ambiguity we explicitly use CommonCode::
const unsigned int CommonCode::NMOS = 1;  /* Number of phases (MOdel Stages) */
const unsigned int CommonCode::NXD  = 5;  /* Number of differential states */
const unsigned int CommonCode::NXA  = 0;  /* Number of algebraic states */
const unsigned int CommonCode::NU   = 1;  /* Number of controls */
const unsigned int CommonCode::NP   = 2;  /* Number of parameters */
const unsigned int CommonCode::NPR  = 0;  /* Number of local parameters */

// *****************************************************************************
// MUSCOD Application
// *****************************************************************************

// Entry point for the muscod application
extern "C" void def_model(void);
void def_model(void) {

  // define problem dimensions
  // NOTE: enforce periodicity on motion
  def_mdims(
    CommonCode::NMOS, CommonCode::NP, CommonCode::RCFCN_N, CommonCode::RCFCN_NE
  );

  // ***************************************************************************
  // Model Stage
  // ***************************************************************************
  unsigned long imos = 0;

  def_mstage(
       imos, // imos,
    // nxd, nxa, nu,
       CommonCode::NXD, CommonCode::NXA, CommonCode::NU,
       NULL, // MayPtr mfcn,
       NULL, // LagPtr lfcn,
    // jacmlo, jacmup, astruc,
       0, 0, 0,
    // MatPtr afcn, RHSPtr ffcn, RHSPtr gfcn,
       NULL, CommonCode::ffcn_avg, NULL,
    // rwh,  iwh
       NULL, NULL
  );

  // define LSQ objective
  def_lsq(imos, "s", CommonCode::NPR,
    CommonCode::LSQFCN_MIN_TAU, CommonCode::lsqfcn_min_tau
  );
  def_lsq(imos, "i", CommonCode::NPR,
    CommonCode::LSQFCN_MIN_TAU, CommonCode::lsqfcn_min_tau
  );
  def_lsq(imos, "e", CommonCode::NPR,
    CommonCode::MSQFCN_TRACK_AVG_WREG_NE, CommonCode::msqfcn_track_avg_wreg
  );

  // define switching behavior for single stage formulation
  def_swt(imos, CommonCode::NSWT,
    &CommonCode::detect_switch_fcn, &CommonCode::execute_switch_fcn_avg
  );

  // define constraints
  // NOTE: enforce periodicity on motion
  def_mpc(
    imos, "s", CommonCode::NPR,
    CommonCode::RDFCN_FEASIBILITY_N, CommonCode::RDFCN_FEASIBILITY_NE,
    CommonCode::rdfcn_feasibility,
    CommonCode::rcfcn_s
  );

  def_mpc(imos, "i", CommonCode::NPR,
    CommonCode::RDFCN_FEASIBILITY_N, CommonCode::RDFCN_FEASIBILITY_NE,
    CommonCode::rdfcn_feasibility,
    NULL
  );

   // NOTE: enforce periodicity on motion at end of motion
  def_mpc(
    imos, "e", CommonCode::NPR,
    CommonCode::RDFCN_FEASIBILITY_N, CommonCode::RDFCN_FEASIBILITY_NE,
    CommonCode::rdfcn_feasibility,
    CommonCode::rcfcn_e
  );

  // increment model stage
  imos++;

  // check number of model stages
  CommonCode::check_dimensions(0, 0, imos, CommonCode::NMOS, __func__);

  // specify setup, tear-down and plotting functions
  // NOTE: data_in is used to calculate current NMOS and NMS
  //       or instantiate different RBLD models for parallel evaluation of
  //       shooting nodes.
  def_mio (CommonCode::data_in , CommonCode::meshup_out, CommonCode::data_out);
}
