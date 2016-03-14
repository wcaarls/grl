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

// Includes from MUSCOD
#include "def_usrmod.hpp"  // providing calls to define models and co
#include "model.hpp"       // providing 'get_pname'

// Standard includes
#include <sstream>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifndef COMMON_CODE_H
#define COMMON_CODE_H

namespace CommonCode { // BEGIN NAMESPACE CommonCode
// *****************************************************************************
// Constants
// *****************************************************************************

	extern const unsigned int NMOS;   // Number of phases (Model stages)
	extern const unsigned int NXD;    // Number of differential states
	extern const unsigned int NXA;    // Number of algebraic states
	extern const unsigned int NU;     // Number of controls
	extern const unsigned int NP;     // Number of parameters
	extern const unsigned int NPR;    // Number of local parameters

	// Number of switching functions
	extern const unsigned int NSWT;
	extern const unsigned int NSWT_DUMMY;

	// dimension constants
	extern const unsigned int RDFCN_FEASIBILITY_N, RDFCN_FEASIBILITY_NE;

	// define square roots of weights for LSQ objective
	// weights
	extern const double w_vref;
	extern const double w_tau;

// define square roots of weights for LSQ objective

	extern const double sw_vref;
	extern const double sw_tau;
	extern const double sw_fp;

	// tolerance of parallelism
	extern const double parallel_TOL;

// *****************************************************************************
// Variables
// *****************************************************************************

	extern char problem_name[];
	extern const std::string model_file_name;

// *****************************************************************************
// External Scope
// *****************************************************************************

	extern "C" {
    extern void set_path(std::string new_problem_path, std::string new_lua_model);

		extern bool get_plot_counter();

		extern void set_plot_counter(bool counter);
	}

// *****************************************************************************
// Utilities
// *****************************************************************************

	// Convenience function to check consistency of variable dimensions
	void check_dimensions(
		const unsigned int n_cnt,  const unsigned int n,
		const unsigned int ne_cnt, const unsigned int ne,
		const std::string func_name = ""
	);

	// convenience functions from MPRL implementation
	inline double get_hip_px(const double& phi_st) {
		return -sin(phi_st);
	}

	inline double get_hip_py(const double& phi_st) {
		return cos(phi_st);
	}

	inline double get_swing_foot_px(const double& phi_st, const double& phi_h) {
		return get_hip_px(phi_st) + sin(phi_st - phi_h);
	}

	inline double get_swing_foot_py(const double& phi_st, const double& phi_h) {
		return get_hip_py(phi_st) - cos(phi_st - phi_h);
	}

	inline double get_hip_vx(const double& phi_st, const double& dphi_st) {
		return -cos(phi_st) * dphi_st;
	}

	inline double get_hip_vy(const double& phi_st, const double& dphi_st) {
		return -sin(phi_st) * dphi_st;
	}

	inline double get_swing_foot_vx(
		const double& phi_st, const double& phi_h,
		const double& dphi_st, const double& dphi_h
	) {
		return get_hip_vx(phi_st, dphi_st) + cos(phi_st - phi_h) * (dphi_st - dphi_h);
	}

	inline double get_swing_foot_vy(
		const double& phi_st, const double& phi_h,
		const double& dphi_st, const double& dphi_h
	) {
		return get_hip_vy(phi_st, dphi_st) + sin(phi_st - phi_h) * (dphi_st - dphi_h);
	}

// *****************************************************************************
// Objective Functions Lagrange and Mayer Type
// *****************************************************************************

// *****************************************************************************
// Objective Functions Least-Squares
// *****************************************************************************

// *****************************************************************************
// Right hand sides
// *****************************************************************************

	// Right-hand side for the single support phase
	void ffcn (
		double *t, double *xd, double *xa, double *u, double *p, double *rhs,
		double *rwh, long *iwh, InfoPtr *info );

	// Right-hand side for the single support phase with augmented states
	void ffcn_avg (
		double *t, double *xd, double *xa, double *u, double *p, double *rhs,
		double *rwh, long *iwh, InfoPtr *info );

// *****************************************************************************
// Implicit State Dependent Switches
// *****************************************************************************

	void detect_switch_fcn (
		double *t, double *xd, double *xa, double* u, double *p,
		long *DUMMY, long *iswt, long *nswt, double *res,
		double *rwh, long *iwh, InfoPtr *info );

	void execute_switch_fcn (
		double *t, double *xd, double *xa, double *u, double *p,
		long iswt, double *rwh, long *iwh, InfoPtr *info
	);

	void execute_switch_fcn_avg (
		double *t, double *xd, double *xa, double *u, double *p,
		long iswt, double *rwh, long *iwh, InfoPtr *info
	);

// *****************************************************************************
// Decoupled Constraints
// *****************************************************************************

	void rdfcn_feasibility (
		double *ts, double *sd, double *sa, double *u, double *p, double *pr,
		double *res, long *dpnd, InfoPtr *info );

// *****************************************************************************
// Coupled Constraints
// *****************************************************************************

// *****************************************************************************
// Data Output
// *****************************************************************************

	void data_in(
		long   *imos,      // index of model stage (I)
		long   *imsn,      // index of m.s. node on current model stage (I)
		double *sd,        // differential states at m.s. node (I/O)
		double *sa,        // algebraic states at m.s. node (I/O)
		double *u,         // controls at m.s. node (I/O)
		double *udot,      // control slopes at m.s. node (I/O)
		double *ue,        // controls at end of m.s. interval (I/O)
		double *uedot,     // control slopes at end of m.s. interval (I/O)
		double *p,         // global model parameters (I/O)
		double *h,         // model stage durations (I/O)
		double *pr         // local i.p.c. parameters (I/O)
	);

	void data_out (
		double *t,         // current time instant
		double *sd,        // differential states at m.s. node (I/O)
		double *sa,        // algebraic states at m.s. node (I/O)
		double *u,         // controls at m.s. node (I/O)
		double *p,         // global model parameters (I/O)
		double *rwh,
		long *iwh,
		InfoPtr *info
	);

	void meshup_out (
		long   *imos,      // index of model stage (I)
		long   *imsn,      // index of m.s. node on current model stage (I)
		double *ts,        // time at m.s. node (I)
		double *te,        // time at end of m.s. interval (I)
		double *sd,        // differential states at m.s. node (I)
		double *sa,        // algebraic states at m.s. node (I)
		double *u,         // controls at m.s. node (I)
		double *udot,      // control slopes at m.s. node (I)
		double *ue,        // controls at end of m.s. interval (I)
		double *uedot,     // control slopes at end of m.s. interval (I)
		double *p,         // global model parameters (I)
		double *pr,        // local i.p.c. parameters (I)
		double *ccxd,
		double *mul_ccxd,  // multipliers of continuity conditions (I)
	#if defined(PRSQP) || defined(EXTPRSQP)
		double *ares,
		double *mul_ares,
	#endif
		double *rd,
		double *mul_rd,    // multipliers of decoupled i.p.c. (I)
		double *rc,
		double *mul_rc,    // multipliers of coupled i.p.c. (I)
		double *obj,
		double *rwh,       // real work array (I)
		long   *iwh        // integer work array (I)
);

} // END NAMESPACE CommonCode

/* COMMON_CODE_H */
#endif
