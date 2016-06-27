// -----------------------------------------------------------------------------
// MUSCOD Problem: Implementing squatting motion of Leo robot
//
// Copyright (c) 2016, Optimization in Robotics and Biomechanics
// (ORB), Heidelberg University, Germany
// All rights reserved
//
// Author(s): Manuel Kudruss <manuel.kudruss@iwr.uni-heidelberg.de>
// -----------------------------------------------------------------------------

// Standard includes
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// MUSCOD includes
#include "def_usrmod.hpp"
#include "model.hpp" // providing 'get_pname'

// RBLD model includes
#include "leomodel.h"

// -----------------------------------------------------------------------------

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

// -----------------------------------------------------------------------------

string path_to_lua_file = "leo_sl.lua";

LeoModel leo;
// std::vector<LeoModel> leo_models;

bool model_loaded = false;
bool has_shown_configuration = false;
bool is_problem_name_initialized = false;
bool add_counter = false;
char problem_name[255];
long real_nmos, real_nmsn;

string rel_data_path = "";

// placeholder for plotting
vector<double> _plotting_t_values;
vector<double> _plotting_p_values;
vector<vector<double> > _plotting_lsq_values;
vector<vector<double> > _plotting_sd_values;
vector<vector<double> > _plotting_u_values;

// -----------------------------------------------------------------------------
// Variables
// -----------------------------------------------------------------------------

const int NMOS = 1;  // Number of phases (MOdel Stages)
const int NXD  = 2*(4);  // Number of differential states
const int NXA  = 0;  // Number of algebraic states

const int NU   = (4);  // Number of controls

const int NP   = 1;  // Number of parameters
const int NPR  = 0;  // Number of local parameters

const int NRC  = 0;  // Number of coupled constraints
const int NRCE = 0;  // Number of coupled equality constraints


map<string, unsigned int> parameter = {
	{"h_ref", 0}
};

map<string, unsigned int> QS = {
	{"ankle_left",  0},
	{"knee_left",   1},
	{"hip_left",    2},
	{"arm",         3},
};

map<string, unsigned int> QDOTS = {
	{"ankle_left",   0 + QS.size()},
	{"knee_left",    1 + QS.size()},
	{"hip_left",     2 + QS.size()},
	{"arm",          3 + QS.size()},
};

map<string, unsigned int> TAUS = {
	{"ankle_left",  0},
	{"knee_left",   1},
	{"hip_left",    2},
	{"arm",         3},
};

// -----------------------------------------------------------------------------
// Utilities
// -----------------------------------------------------------------------------

  // Hack to allow external programs to alter relative path of DAT files
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

// Convenience function to check consistency of variable dimensions
void check_dimensions(
		const unsigned int n_cnt,  const unsigned int n,
		const unsigned int ne_cnt, const unsigned int ne,
		const string func_name
){
	if ((n_cnt != n) || (ne_cnt != ne)) {
		if (func_name != "") {
			cout << "Dimensions of function '" << func_name;
			cout << "' are inconsistent!" << endl;
		}
		cout << "n_cnt:  " << n_cnt  << " (" << n  << ")" << endl;
		cout << "ne_cnt: " << ne_cnt << " (" << ne << ")" << endl;
		cout << endl;
		cout << "bailing out..." << endl;
		abort();
	}
	return;
}

// -----------------------------------------------------------------------------
// Least Squares Functions
// -----------------------------------------------------------------------------

const unsigned int LSQFCN_HEIGHT_TRACKING_NE = 0
	+ 1 //stability tracking of center of support
	+ 1 // com velocity minimization
	+ 1 // height tracking
	+ 1 // height change minimization
	+ 1 // minimize angular momentum
	+ 1 // upright torso
	+ 1 // regularizing arm motion
	;
void lsqfcn_height_tracking (
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
	// define rhs counter
	unsigned int res_cnt = 0;

	// update model with current states
	// NOTE: for activeConstraintSet of "" ABA is used directly
	leo.updateState (sd, u, p, "");

	// retrieve
	Vector3d position_tip_left = leo.getPointPosition("tip_left");
	Vector3d position_heel_left = leo.getPointPosition("heel_left");

	// calculate support center from feet positions
	Vector3d suppport_center = Vector3d::Zero();
	suppport_center += position_tip_left;
	suppport_center += position_heel_left;
	suppport_center /= 2.0;

	// retrieve center of root body
	Vector3d position_root = leo.getPointPosition("root");

	// retrieve CoM from model
	Vector3d position_CoM = leo.calcCenterOfMass();
	Vector3d velocity_CoM = leo.calcCenterOfMassVelocity();
	Vector3d momentum_CoM = leo.calcAngularMomentum();

	// if (false) {
	// 	std::cout << "In function " << __func__ << " at time t " << *ts << std::endl;
	// 	std::cout << "position_CoM:        " << position_CoM.transpose() << std::endl;
	// 	std::cout << "position_root:       " << position_root.transpose() << std::endl;
	// 	std::cout << "momentum_CoM:        " << momentum_CoM.transpose() << std::endl;
	// 	std::cout << "suppport_center:     " << suppport_center.transpose() << std::endl;
	// 	// std::cout << "position_tip_left:   " << position_tip_left.transpose() << std::endl;
	// 	// std::cout << "position_heel_left:  " << position_tip_left.transpose() << std::endl;

	// 	std::cout << position_heel_left[0] << " <= " << position_CoM[0] << " <= " << position_tip_left[0] << std::endl;

	// 	std::cout << std::endl;
	// }

	// define the residual r(t,xd,xa,u,p) for a least-squares problem of the form
	//    L = ||r(t,xd,xa,u,p,pr)||_2^2
	// here, according to:
	//    r(t,xd,xa,u,p) = [sqrt(w_0) * r_0(t,xd,xa,u,p)]
	//                     [sqrt(w_1) * r_1(t,xd,xa,u,p)]
	//                     [                        ... ]
	//                     [sqrt(w_n) * r_n(t,xd,xa,u,p)]
	// NOTE: In C++ the '++' operator as postfix first evaluates the variable
	//       and then increments it, i.e.
	//       n = 0;    -> n  = 0
	//       n2 = n++; -> n2 = 0, n = 1

	// track: || root_z - h_ref ||_2^2
	res[res_cnt++] =  100.0 * (position_root[2] - p[parameter["h_ref"]]);

	// track: || com_x,y - support center_x,y ||_2^2
	res[res_cnt++] =   10.00 * (position_CoM[0] - suppport_center[0]);

	res[res_cnt++] =   10.00 * velocity_CoM[0];
	res[res_cnt++] =   10.00 * velocity_CoM[2];

	res[res_cnt++] =    10.0 * momentum_CoM[1];
	// res[res_cnt++] = 10.0 * momentum_CoM[0];
	// res[res_cnt++] = 10.0 * momentum_CoM[0];

	// NOTE: sum of lower body angles is equal to angle between ground slope
	//       and torso. Minimizing deviation from zero keeps torso upright
	//       during motion execution.
	res[res_cnt++] = 1.00 * (
		sd[QS["hip_left"]]
		+ sd[QS["knee_left"]]
		+ sd[QS["ankle_left"]]
	);

	// regularize: || q - q_desired ||_2^2
  res[res_cnt++] = 1 * (sd[QS["arm"]]         - (-0.26)); // arm
	// res[res_cnt++] = 1.00 * (sd[QS["hip_left"]]    - (-0.01)); // hip_left
	// res[res_cnt++] = 0.01 * (sd[QS["knee_left"]]   - (-0.01)); // knee_left
	// res[res_cnt++] = 0.01 * (sd[QS["ankle_left"]]  - (0.05)); // ankle_left

	// regularize: || qdot ||_2^2
	// res[res_cnt++] = 0.10 * leo.qdot[QS["arm"]]         * leo.qdot[QS["arm"]]; // arm
	// res[res_cnt++] = 0.10 * leo.qdot[QS["hip_left"]]    * leo.qdot[QS["hip_left"]]; // hip_left
	// res[res_cnt++] = 0.10 * leo.qdot[QS["knee_left"]]   * leo.qdot[QS["knee_left"]]; // knee_left
	// res[res_cnt++] = 0.10 * leo.qdot[QS["ankle_left"]]  * leo.qdot[QS["ankle_left"]]; // ankle_left

	// regularize: || u ||_2^2
	// res[res_cnt++] = 0.10 * u[TAUS["arm"]]         * u[TAUS["arm"]]; // arm
	// res[res_cnt++] = 0.10 * u[TAUS["hip_left"]]    * u[TAUS["hip_left"]]; // hip_left
	// res[res_cnt++] = 0.01 * u[TAUS["knee_left"]]   * u[TAUS["knee_left"]]; // knee_left
	// res[res_cnt++] = 0.10 * u[TAUS["ankle_left"]]  * u[TAUS["ankle_left"]]; // ankle_left

	// regularize: || tau ||_2^2
	// res[res_cnt++] = 0.010 * leo.tau[TAUS["arm"]]         * leo.tau[TAUS["arm"]]; // arm
	// res[res_cnt++] = 0.001 * leo.tau[TAUS["hip_left"]]    * leo.tau[TAUS["hip_left"]]; // hip_left
	// res[res_cnt++] = 0.001 * leo.tau[TAUS["knee_left"]]   * leo.tau[TAUS["knee_left"]]; // knee_left
	// res[res_cnt++] = 0.001 * leo.tau[TAUS["ankle_left"]]  * leo.tau[TAUS["ankle_left"]]; // ankle_left

	// // regularize: || qdot * tau ||_2^2
	// res[res_cnt++] = 0.01 * leo.tau[TAUS["arm"]]        * leo.qdot[QDOTS["arm"]];
	// res[res_cnt++] = 0.01 * leo.tau[TAUS["hip_left"]]   * leo.qdot[QDOTS["hip_left"]];
	// res[res_cnt++] = 0.01 * leo.tau[TAUS["knee_left"]]  * leo.qdot[QDOTS["knee_left"]];
	// res[res_cnt++] = 0.01 * leo.tau[TAUS["ankle_left"]] * leo.qdot[QDOTS["ankle_left"]];

	// check dimensions of functions
	check_dimensions(0, 0, res_cnt, LSQFCN_HEIGHT_TRACKING_NE, __func__);
}

// -----------------------------------------------------------------------------
// Right Hand Sides
// -----------------------------------------------------------------------------

void ffcn (
	double *t, double *xd, double *xa, double *u, double *p, double *rhs,
	double *rwh, long *iwh, InfoPtr *info
) {
	// update state of model
	leo.updateState (xd, u, p, "");

	// evaluate forward dynamics
	leo.calcForwardDynamicsRhs (rhs);
}

// -----------------------------------------------------------------------------
// Decoupled Constraints
// -----------------------------------------------------------------------------

/// \brief Constraints at the phase start point
const unsigned int RDFCN_N = 2, RDFCN_NE = 0;
void rdfcn(
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
	unsigned int res_n_cnt  = 0;
	unsigned int res_ne_cnt = 0;

	// update model with current states
	Vector3d CoM = leo.calcCenterOfMass();

	// retrieve contact points
	Vector3d position_tip_left = leo.getPointPosition("tip_left");
	Vector3d position_heel_left = leo.getPointPosition("heel_left");

	// retrieve contact forces
	// TODO get contact forces of fixed leg
	// Vector3d force_tip_left = leo.getPointForce("tip_left");
	// Vector3d force_heel_left = leo.getPointForce("heel_left");

	// if (false) {
		// std::cout << "CoM:                 " << CoM.transpose() << std::endl;
		// std::cout << "position_tip_left:   " << position_tip_left.transpose() << std::endl;
		// std::cout << "position_heel_left:  " << position_heel_left.transpose() << std::endl;
		// std::cout << "position_tip_right:  " << position_tip_right.transpose() << std::endl;
		// std::cout << "position_heel_right: " << position_heel_right.transpose() << std::endl;

		// std::cout << position_heel_left[0] << " <= " << CoM[0] << " <= " << position_tip_left[0] << std::endl;
		// std::cout << position_heel_right[1] << " <= " << CoM[1] << " <= " << position_heel_left[1] << std::endl;

		// std::cout << "force_tip_left:   " << force_tip_left.transpose() << std::endl;
		// std::cout << "force_heel_left:  " << force_tip_left.transpose() << std::endl;
		// std::cout << "force_tip_right:  " << force_tip_right.transpose() << std::endl;
		// std::cout << "force_heel_right: " << force_heel_right.transpose() << std::endl;
		// std::cout << std::endl;
	// }

	// define constraint residuals here, i.e. res[:_ne] = 0 and res[_ne:] >= 0!
	// NOTE: In C++ the '++' operator as postfix first evaluates the variable
	//       and then increments it, i.e.
	//       n = 0;    -> n  = 0
	//       n2 = n++; -> n2 = 0, n = 1

	// NOTE: For a standing motion CoM has to stay in support polygon
	//       foot_left_x,y <= com <= right_foot_x,y
	res[res_n_cnt++] = position_tip_left[0] - CoM[0]; // >= 0
		// min(position_tip_right[0], position_heel_right[0]) - CoM[0]; // >= 0
	res[res_n_cnt++] = CoM[0] - position_heel_left[0]; // >= 0
		// CoM[0] - max(position_tip_left[0], position_heel_left[0]); // >= 0

	// NOTE: For a standing motion all contact forces have to be positive
	// TODO: get contact forces for fixed foot
	// res[res_n_cnt++] = force_tip_left[2]; // >= 0
	// res[res_n_cnt++] = force_heel_left[2]; // >= 0
	// res[res_n_cnt++] = force_tip_right[2]; // >= 0
	// res[res_n_cnt++] = force_heel_right[2]; // >= 0

	// NOTE: For a standing motion have to stay in a friction cone (mu = 0.8?)
	// TODO: get contact forces for fixed foot

	// check dimensions of constraints
	check_dimensions(
		res_n_cnt, RDFCN_N, res_ne_cnt, RDFCN_NE, __func__
	);
}

// -----------------------------------------------------------------------------
// Data Input
// -----------------------------------------------------------------------------

void data_in (
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
	if (!has_shown_configuration) {
		cout << "MODEL CONFIGURATION" << endl;
		cout << endl;
		cout << "NMOS = " << NMOS << endl;
		cout << "NP   = " << NP   << endl;
		cout << "NRC  = " << NRC  << endl;
		cout << "NRCE = " << NRCE << endl;
		cout << endl;
		cout << "NXD  = " << NXD  << endl;
		cout << "NXA  = " << NXA  << endl;
		cout << "NU   = " << NU   << endl;
		cout << "NPR  = " << NPR  << endl;

		has_shown_configuration = true;
	}

	// count model stages and shooting nodes
	real_nmos = max(real_nmos, *imos);
	real_nmsn = max(real_nmsn, *imsn);

	// load Leo model from lua file
	bool verbose = false;
	if (!model_loaded) {
		// LeoModel leo;
    string lua_path = rel_data_path + '/' + path_to_lua_file;
    leo.loadModelFromFile (lua_path.c_str(), verbose);
    leo.loadPointsFromFile (lua_path.c_str(), verbose);
    leo.loadConstraintSetsFromFile (lua_path.c_str(), verbose);
		model_loaded = true;

		// assign values from model
		assert (NP == parameter.size());
		assert (NXD == 2*leo.nDof);
		assert (NU == leo.nActuatedDof);
		assert (leo.nDof == leo.nActuatedDof);

		assert (NXD == QS.size() + QDOTS.size());
		assert (QS.size() == QDOTS.size());
	}
	// setup RBDL models for each shooting node! for each shooting node!
	// leo_models.push_back(leo);
}

// -----------------------------------------------------------------------------
// Data Output
// -----------------------------------------------------------------------------

void data_out(
	double *t, double *sd, double *sa, double *u, double *p,
	double *rwh, long *iwh, InfoPtr *info
	) {
	long lnode = -1;
	long iteration_cnt = 0;
		// get problem name from MUSCOD
		// NOTE: fixed size array could cause problems for larger problem names
	if (!is_problem_name_initialized) {
		memset (problem_name, 0, 255);
		get_pname (problem_name);
	}

	if (*t == 0.) {
		ofstream meshup_csv_stream;

		// crate string with purpose suffix
		string meshup_header_file = string("RES/meshup_");
		string data_lsq_file      = string("lsq_");
		string data_sd_file       = string("sd_");
		string data_u_file        = string("u_");
		string data_p_file        = string("p_");

		// add problem name identifier
		meshup_header_file += string(problem_name);
		data_lsq_file      += string(problem_name);
		data_sd_file       += string(problem_name);
		data_u_file        += string(problem_name);
		data_p_file        += string(problem_name);

		// add iteration counter [optional]
		if (add_counter) {
			stringstream sstm;
			sstm << string("_");
			// NOTE: equivalent to "%03d"
			sstm << setfill('0') << setw(4) << iteration_cnt;
			meshup_header_file += sstm.str();
			data_lsq_file      += sstm.str();
			data_sd_file       += sstm.str();
			data_u_file        += sstm.str();
			data_p_file        += sstm.str();
		}

		// add file identifier suffix
		meshup_header_file += string(".csv");
		data_lsq_file      += string(".csv");
		data_sd_file       += string(".csv");
		data_u_file        += string(".csv");
		data_p_file        += string(".csv");

		ofstream meshup_header_stream;
		ofstream data_lsq_stream;
		ofstream data_sd_stream;
		ofstream data_u_stream;
		ofstream data_p_stream;

		meshup_header_stream.open (meshup_header_file.c_str(),               ios_base::trunc);
		data_lsq_stream.     open ((string("RES/") + data_lsq_file).c_str(), ios_base::trunc);
		data_sd_stream.      open ((string("RES/") + data_sd_file).c_str(),  ios_base::trunc);
		data_u_stream.       open ((string("RES/") + data_u_file). c_str(),  ios_base::trunc);
		data_p_stream.       open ((string("RES/") + data_p_file). c_str(),  ios_base::trunc);

		if (
			!meshup_header_stream || !data_lsq_stream || !data_sd_stream ||
			!data_u_stream || !data_p_stream
		) {
			cerr << "Error opening file ";
			if (!meshup_header_stream) {
				cerr << meshup_header_file;
			}
			if (!data_sd_stream) {
				cerr << data_sd_file;
			}
			if (!data_u_stream) {
				cerr << data_u_file;
			}
			if (!data_p_stream) {
				cerr << data_p_file;
			}
			cerr << endl;
			abort();
		}
		const char* meshup_header = "";
		meshup_header_stream << meshup_header << endl;
		meshup_header_stream << "DATA_FROM: " << data_sd_file << endl;

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
				data_sd_stream << endl;

				// controls
				data_u_stream << _plotting_t_values[i] << ", ";
				for (unsigned int j = 0; j < _plotting_u_values[i].size(); j++) {
					data_u_stream << _plotting_u_values[i][j];
					if (j < _plotting_u_values[i].size() -1 )
						data_u_stream << ", ";
				}
				data_u_stream << endl;
			}

			// save time independent quantities
			// parameters
			for (unsigned int j = 0; j < _plotting_p_values.size(); j++) {
				data_p_stream << _plotting_p_values[j];
				if (j < _plotting_p_values.size() -1 )
					data_p_stream << ", ";
			}
			data_p_stream << endl;

			iteration_cnt++;
		}

		_plotting_t_values.clear();
		_plotting_lsq_values.clear();
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

	vector<double> sd_vec (NXD);
	for (unsigned i = 0; i < NXD; i++)
		sd_vec[i] = sd[i];
	_plotting_sd_values.push_back (sd_vec);

	vector<double> u_vec (NU);
	for (unsigned i = 0; i < NU; i++)
		u_vec[i] = u[i];
	_plotting_u_values.push_back (u_vec);
}

void meshup_out
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

// -----------------------------------------------------------------------------
// MUSCOD Application
// -----------------------------------------------------------------------------

// Entry point for the muscod application
extern "C" void def_model (void);
void def_model (void)
{
	// define problem dimensions
	def_mdims(NMOS, NP, NRC, NRCE);

	//  ------------------------------------------------------------------------
	// Model Stage
	//  ------------------------------------------------------------------------
	unsigned long imos = 0;

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
	// NOTE:
	def_lsq(imos, "c", NPR,
		LSQFCN_HEIGHT_TRACKING_NE, lsqfcn_height_tracking
	);

	// define constraints
	def_mpc(imos, "*", NPR, RDFCN_N, RDFCN_NE, rdfcn, NULL);

	// increment model stage
	imos++;

	// check number of model stages
	check_dimensions(0, 0, imos, NMOS, __func__);

	// specify setup, tear-down and plotting functions
	// NOTE: data_in is used to calculate current NMOS and NMS
	//       or instantiate different RBLD models for parallel evaluation of
	//       shooting nodes.
	def_mio (data_in , meshup_out, data_out);
}
// -----------------------------------------------------------------------------
