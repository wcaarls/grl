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

// define problem name
static const std::string PROBLEM_NAME = "Passive NMPC Stages";

// define MUSCOD-II Dimensions
static const unsigned int NMOS = 6;  // Number of phases (MOdel Stages)
static const unsigned int NXD  = 5;  // Number of differential states
static const unsigned int NXA  = 0;  // Number of algebraic states
static const unsigned int NU   = 1;  // Number of controls
static const unsigned int NP   = 6;  // Number of parameters
static const unsigned int NPR  = 0;  // Number of local parameters

// tolerance of parallelism
static const double parallel_TOL = 0.15;

// define square roots of weights for LSQ objective
// RL discount factor
static const double rl_y = 1.000;
static const double rl_sy = sqrt(rl_y);

// define weight constant for limit cycle
static const double K = 1.0; // number of future limit cycles
static const double sK = sqrt(K);

// weights
static const double w_vref = 10.00;
static const double w_tau =  00.10;
static const double w_time = 01.00;
static const double w_periodicity = 00.01;
static const double w_slacks = 1.00;
static const double w_slackf = 10.00;

// define square roots of weights for LSQ objective
static const double sw_vref = sqrt(w_vref);
static const double sw_tau = sqrt(w_tau);
static const double sw_time = sqrt(w_time);
static const double sw_periodicity = sqrt(w_periodicity);
static const double sw_slacks = sqrt(w_slacks);
static const double sw_slackf = sqrt(w_slackf);

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

// include common code
#include "common_code.cpp"

// *****************************************************************************
// MUSCOD Application
// *****************************************************************************

// Entry point for the muscod application
extern "C" void def_model(void);
void def_model(void) {

	// define problem dimensions
	// NOTE: enforce periodicity on motion
	def_mdims(
		NMOS, NP, RCFCN_N, RCFCN_NE
	);

	unsigned long imos = 0;
	// ***************************************************************************
	// Model Stage: Feedback Stage
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

	// define LSQ objective
	def_lsq(imos, "*", NPR, LSQFCN_MIN_TAU, lsqfcn_min_tau);

	// define constraints
	// NOTE because of shrinking horizon and foot impact at stage end
	//      no constraints can be specified
	def_mpc(
		imos, "s", NPR,
		RDFCN_PARTLY_FEASIBILITY_N, RDFCN_PARTLY_FEASIBILITY_NE,
		rdfcn_partly_feasibility,
		// RDFCN_SLACKED_FEASIBILITY_N, RDFCN_SLACKED_FEASIBILITY_NE,
		// rdfcn_slacked_feasibility,
		NULL
	);

	// increment model stage
	imos++;

	// ***************************************************************************
	// Model Stage: Single Support
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

	// define LSQ objective
	def_lsq(imos, "s", NPR, LSQFCN_MIN_TAU_AND_MAX_TIME, lsqfcn_min_tau_and_max_time);
	def_lsq(imos, "i", NPR, LSQFCN_MIN_TAU, lsqfcn_min_tau);

	// define constraints
	// NOTE because of shrinking horizon and foot impact at stage end
	//      no constraints can be specified
	def_mpc(
		imos, "s", NPR,
		RDFCN_PARTLY_FEASIBILITY_N, RDFCN_PARTLY_FEASIBILITY_NE,
		rdfcn_partly_feasibility,
		NULL
	);

	def_mpc(
		imos, "i", NPR,
		RDFCN_PARTLY_FEASIBILITY_N, RDFCN_PARTLY_FEASIBILITY_NE,
		rdfcn_partly_feasibility,
		NULL
	);

	// increment model stage
	imos++;

	// ***************************************************************************
	// Model Stage: Double Support
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
		NULL, ffcn_trans, NULL,
		// rwh,  iwh
		NULL, NULL
	);

	// define LSQ objective
	// NOTE: define min tau to regularize controls
	def_lsq(imos, "*", NPR,
		LSQFCN_MIN_TAU, lsqfcn_min_tau
	);

	// define constraints
	// NOTE: enforce switching conditions before impact
	def_mpc(imos, "s", NPR,
		RDFCN_TRANS_N, RDFCN_TRANS_NE,
		rdfcn_trans,
		NULL
	);

	// def_mpc(
	// 	imos, "i", NPR,
	// 	RDFCN_FULL_FEASIBILITY_N, RDFCN_FULL_FEASIBILITY_NE,
	// 	rdfcn_full_feasibility,
	// 	NULL
	// );

	// increment model stage
	imos++;

	// ***************************
	// Model Stage: Parallel Feedback Phase in limit cycle
	// ***************************

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

	// define LSQ objective
	def_lsq(imos, "*", NPR, LSQFCN_MIN_TAU, lsqfcn_min_tau);

	// define constraints
	// NOTE: enforce periodicity on motion
	def_mpc(
		imos, "s", NPR,
		0, 0, NULL,
		// RDFCN_FULL_FEASIBILITY_N, RDFCN_FULL_FEASIBILITY_NE,
		// rdfcn_full_feasibility,
		// RDFCN_PARTLY_FEASIBILITY_N, RDFCN_PARTLY_FEASIBILITY_NE,
		// rdfcn_partly_feasibility,
		rcfcn_s
	);

	// def_mpc(
	// 	imos, "i", NPR,
	// 	RDFCN_FULL_FEASIBILITY_N, RDFCN_FULL_FEASIBILITY_NE,
	// 	rdfcn_full_feasibility,
	// 	NULL
	// );

	// increment model stage
	imos++;

	// ***************************
	// Model Stage: Single Support
	// ***************************

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

	// define LSQ objective
	def_lsq(imos, "*", NPR, LSQFCN_MIN_TAU, lsqfcn_min_tau);

	// define constraints
	def_mpc(
		imos, "s", NPR,
		0, 0, NULL,
		// RDFCN_FULL_FEASIBILITY_N, RDFCN_FULL_FEASIBILITY_NE,
		// rdfcn_full_feasibility,
		NULL
	);

	def_mpc(
		imos, "i", NPR,
		0, 0, NULL,
		// RDFCN_FULL_FEASIBILITY_N, RDFCN_FULL_FEASIBILITY_NE,
		// rdfcn_full_feasibility,
		NULL
	);

	// increment model stage
	imos++;

	// ***************************************************************************
	// Model Stage: Double Support
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
		NULL, ffcn_trans, NULL,
		// rwh,  iwh
		NULL, NULL
	);

	// define LSQ objective
	// NOTE: define min tau to regularize controls
	def_lsq(imos, "s", NPR, LSQFCN_MIN_TAU, lsqfcn_min_tau);
	def_lsq(imos, "i", NPR, LSQFCN_MIN_TAU, lsqfcn_min_tau);
	def_lsq(imos, "e", NPR, LSQFCN_MIN_SLACKS, lsqfcn_min_slacks);

	// define constraints
	// NOTE: enforce switching conditions before impact
	def_mpc(imos, "s", NPR,
		RDFCN_TRANS_N, RDFCN_TRANS_NE,
		rdfcn_trans,
		NULL
	);

	def_mpc(
		imos, "i", NPR,
		0, 0, NULL,
		// RDFCN_FULL_FEASIBILITY_N, RDFCN_FULL_FEASIBILITY_NE,
		// rdfcn_full_feasibility,
		NULL
	);

	// NOTE: enforce periodicity on motion at end of motion
	def_mpc(
		imos, "e", NPR,
		0, 0, NULL,
		// RDFCN_FULL_FEASIBILITY_N, RDFCN_FULL_FEASIBILITY_NE,
		// rdfcn_full_feasibility,
		rcfcn_e
	);

	// increment model stage
	imos++;

	// check number of model stages
	// check_dimensions(0, 0, imos, NMOS, __func__);

	// specify setup, tear-down and plotting functions
	// NOTE: data_in is used to calculate current NMOS and NMS
	//       or instantiate different RBLD models for parallel evaluation of
	//       shooting nodes.
	def_mio (data_in, NULL, NULL);
	// def_mio (data_in , meshup_out, data_out);
}
