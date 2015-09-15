//****************************************************************************
// MUSCOD-II example
//
// Implementation of pendulum on a cart using RBDL
//
// Copyright (C) 2015, Manuel Kudruss, IWR, Heidelberg
//
//*****************************************************************************

#include <cmath>

#include "def_usrmod.hpp"
#include "model.hpp"

#include <iomanip>
#include <limits>
#include <vector>
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#include "rbdl/rbdl.h"
#include <rbdl/addons/luamodel/luamodel.h>
#include <rbdl/addons/luamodel/luatables.h>
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;

// Global Definitions
unsigned int nDof;
unsigned int nActuatedDof;

// Variables
Model *model = NULL;
string model_file_name  ="./model.lua";
char problem_name[255];
std::string rel_data_path = "";

VectorNd Q, QDOT, QDDOT, TAU;

int NMOS   = -1;  /* Number of phases (MOdel Stages) */
int NP     =  1;  /* Number of parameters */
int NRC    =  0;  /* Number of coupled constraints */
int NRCE   =  0;  /* Number of coupled equality constraints */

int NXD    = -1;  /* Number of differential states */
int NXA    =  0;  /* Number of algebraic states */
int NU     = -1;  /* Number of controls */
int NPR    =  0;  /* Number of local parameters */

// ********************************o
// *
// * Variables
// *
// ********************************o

std::vector<double> xref_time;
std::vector<std::vector<double> > xref_states;
std::vector<std::vector<double> > xref_cntrls;

std::ifstream xref_states_f;
std::ifstream xref_cntrls_f;

// Optimal trajectory and control
std::string xref_states_path = "sd_cart_pendulum_time_pc.csv";
std::string xref_cntrls_path = "u_cart_pendulum_time_pc.csv";

// constants
const double cart_m =  10.0;
const double pend_m =   1.0;
const double pend_l =   0.5;
const double gravty = -9.81;

const double mu0 = 0.01;
const double mu1 = 0.01;

// weights
const double w0 =  1.0000;
const double w1 = 10.0000;
const double w2 =  0.1000;
const double w3 =  0.1000;
const double w4 =  0.0001;

const double sw0 = sqrt(w0);
const double sw1 = sqrt(w1);
const double sw2 = sqrt(w2);
const double sw3 = sqrt(w3);
const double sw4 = sqrt(w4);

// ********************************o
// *
// * Utilities
// *
// ********************************o
double angleCnt = M_PI; // initially start from the bottom
double angleOld = 0;    // old value of mprl angle
/**
 * Function which maps angle from MPRL representation to MUSCOD representation
 * @param angle - input angle wrapped around [0; 2*pi] where 0 or 2*pi mean bottom position, pi - upright
 * @return MUSCOD continuous angle where 2*pi*k mean upright position, pi*k - bottom,
 * and k is a period, ...,-1,0,1,2,...
 */
double mapAngleToMuscod(double angle){
  int max_k = 1; // consider only +/- 1 period change
  double delta = 2*M_PI;
  for (int k = -max_k; k <= max_k; k++)
    if (fabs(angle-angleOld + k*2*M_PI) < fabs(delta))
      delta = angle-angleOld + k*2*M_PI;

  angleCnt += delta;
  angleOld = angle;
  return angleCnt;
}

extern "C" {
  void set_path(std::string new_path){
    rel_data_path = new_path + "/";
    std::cout << "NMPC: setting new data path to: '" << rel_data_path << "'" <<std::endl;
  }
  /**
   * Function which converts a wrapped MPRL state into continuous MUSCOD state
   * Use input vectors of size 0 to reinitialize (e.g. when you restart environment)
   * @param from: wrapped MPRL state
   * @param to: continuous MUSCOD state
   */
  void convert_obs_for_muscod(const std::vector<double> &from, std::vector<double> &to){
    if (from.size() == 0 && to.size() == 0)
    {
      // initialize
      angleCnt = M_PI;
      angleOld = 0;
      std::cout << "NMPC: Observation converter is initialized." <<std::endl;
      return;
    }
    to[0] = from[0];
    to[1] = mapAngleToMuscod(from[1]);
    to[2] = from[2];
    to[3] = from[3];
  }
}

void xref_setup(){
    std::string line;

    // add a relative path
    std::string xref_states_path_full = rel_data_path + xref_states_path;
    std::string xref_cntrls_path_full = rel_data_path + xref_cntrls_path;

    // load file
    xref_states_f.open(xref_states_path_full.c_str());

    // process file
    if (xref_states_f.is_open()) {
      while (std::getline(xref_states_f, line)) {
        // Insert the string into a stream
        std::istringstream ss(line);

        // process entries per line
        double dummy;
        std::vector<double> values;
        while(ss >> dummy){
          values.push_back(dummy);

          // state exceptions to ignore
          if (ss.peek() == ',' || ss.peek() == ' ') {
            ss.ignore();
          }
        }

        // Store values in appropriate container
        xref_time.push_back(values[0]);
        values.erase(values.begin()); // remove first entry

        xref_states.push_back(values);
      }
    } else {
      std::cerr << "ERROR: Could not open file: " << xref_states_path_full << std::endl;
      std::exit(-1);
    }
    xref_states_f.close();

    // load file
    xref_cntrls_f.open(xref_cntrls_path_full.c_str());
    // process file
    if (xref_cntrls_f.is_open()) {
      while (std::getline(xref_cntrls_f, line)) {
        // Insert the string into a stream
        std::istringstream ss(line);

        // process entries per line
        double dummy;
        std::vector<double> values;
        while(ss >> dummy){
          values.push_back(dummy);

          // state exceptions to ignore
          if (ss.peek() == ',' || ss.peek() == ' ') {
            ss.ignore();
          }
        }

        // Store values in appropriate container
        values.erase(values.begin()); // remove first entry
        xref_cntrls.push_back(values);
      }
    } else {
      std::cerr << "ERROR: Could not open file: " << xref_cntrls_path_full << std::endl;
      std::exit(-1);
    }
    xref_cntrls_f.close();
}

/******************************************************************************/

void closest(std::vector<double>& vec, const double& value, int& index) {
  std::vector<double>::iterator it;
  it = std::lower_bound(vec.begin(), vec.end(), value);
  if (it == vec.end()) {
    index = -1;
  } else {
    index = std::distance(vec.begin(), it);
  }
}

/******************************************************************************/

double xref_xd(const int choice, const double t){
  int index;
  // find closest
  closest(xref_time, t, index);
  if(index<0){
    return 0.0;
  } else {
    std::vector<double> state = xref_states[index];
    return state[choice];
  }
}

double xref_u(const int choice, const double t){
  int index;
  // find closest
  closest(xref_time, t, index);
  if(index<0){
    return 0.0;
  } else {
    std::vector<double> cntrl = xref_cntrls[index];
    return cntrl[choice];
  }
}

/******************************************************************************/

// update RBDL vectors
static void updateState (const double *sd, const double *u=NULL) {
    Q   [0] = sd[0];
    Q   [1] = sd[1];
    QDOT[0] = sd[2];
    QDOT[1] = sd[3];

    TAU[0]  = 0.0;
    TAU[1]  = 0.0;

    if (u)
    {
        TAU[0] = u[0];
    }
}

// ********************************o
// *
// * Objective Funtions
// *
// ********************************o

/** \brief Objective function (Lagrangian type) */
static void lfcn_tracking(double *t, double *xd, double *xa, double *u,
  double *p, double *lval, double *rwh, long *iwh, InfoPtr *info) {

  // integrate over constant
    *lval = 0.0;

  // define objective
  *lval += w0 * ( xd[0] ) * ( xd[0] ); // = 0
  *lval += w1 * ( xd[1] ) * ( xd[1] ); // = 0
  *lval += w2 * ( xd[2] ) * ( xd[2] ); // = 0
  *lval += w3 * ( xd[3] ) * ( xd[3] ); // = 0
  *lval += w4 * (  u[0] ) * (  u[0] ); // = 0
}

/** \brief Objective function (Mayer type) */
static void mfcn_penalty(double *ts, double *xd, double *xa, double *p, double *pr,
    double *mval,  long *dpnd, InfoPtr *info) {
  if (*dpnd) {
      *dpnd = MFCN_DPND(0, *xd, 0, 0, 0);
      return;
  }
  // zeroize Mayer
  *mval = 0.0;

  *mval +=  w0 * ( xd[0] ) * ( xd[0] ); // = 0
  *mval +=  w1 * ( xd[1] ) * ( xd[1] ); // = 0
  *mval +=  w2 * ( xd[2] ) * ( xd[2] ); // = 0
  *mval +=  w3 * ( xd[3] ) * ( xd[3] ); // = 0
}

static void lsqfcn(double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
  if (*dpnd) { *dpnd = RFCN_DPND(0, *sd, 0, *u, 0, 0); return; }
  // res = r(t,x,u,p)
  // L   = ||r(x)||_2^2

  // get real_time from nmpc_loop
  double real_ts = *ts + p[0];

  // res = sqrt(w) * quantity
  res[0] = sw0 * ( sd[0] - xref_xd(0, real_ts) );
  res[1] = sw1 * ( sd[1] - xref_xd(1, real_ts) );
  res[2] = sw2 * ( sd[2] - xref_xd(2, real_ts) );
  res[3] = sw3 * ( sd[3] - xref_xd(3, real_ts) );
  //res[4] = sw4 * (  u[0] - xref_u (0, real_ts) );
}

static void msqfcn(double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
  if (*dpnd) { *dpnd = RFCN_DPND(0, *sd, 0, *u, 0, 0); return; }
  // res = r(t,x,u,p)
  // L   = ||r(x)||_2^2
  // res = sqrt(w) * quantity
  res[0] = sd[0]; //sw0 * sd[0];
  res[1] = sd[1]; //sw1 * sd[1];
  res[2] = sd[2]; //sw2 * sd[2];
  res[3] = sd[3]; //sw3 * sd[3];
  res[4] =  u[0]; //sw4 *  u[0];
}

// ********************************o
// *
// * Right Hand Sides
// *
// ********************************o

static void ffcn (double *t, double *xd, double *xa, double *u,
  double *p, double *rhs, double *rwh, long *iwh, InfoPtr *info)
{
    updateState (xd, u);
    ForwardDynamics (*model, Q, QDOT, TAU, QDDOT);

    for (unsigned int i = 0; i < nDof; i++) {
        rhs[i]        = QDOT [i];
        rhs[i + nDof] = QDDOT[i];
    }
}

// ********************************o
// *
// * Decoupled Constraints
// *
// ********************************o

//         # of all constraints       # of equality constraints
static int RDFCN_S_N = 4, RDFCN_S_NE = 4;
static void rdfcn_s(double *ts, double *sd, double *sa, double *u,
        double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
    if (*dpnd) {
        *dpnd = RFCN_DPND(NULL, *sd, 0, 0, 0, 0);
        return;
    }
    updateState (sd, NULL);
    res[0] = (        Q[0] ) * 100; // = 0
    res[1] = ( 3.14 - Q[1] ) * 100; // = 0
    res[2] = (     QDOT[0] ) * 100; // = 0
    res[3] = (     QDOT[1] ) * 100; // = 0
}

/// \brief Constraints at end point */
static int RDFCN_E_N = 4, RDFCN_E_NE = 0;
static void rdfcn_e(double *ts, double *sd, double *sa, double *u,
        double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
    if (*dpnd) {
        *dpnd = RFCN_DPND(NULL, *sd, 0, 0, *p, 0);
        return;
    }
    updateState (sd, NULL);
    res[0] = 1.0*(    Q[0] *    Q[0] ) - p[0]; // >= 0
    res[1] = 1.0*(    Q[1] *    Q[1] ) - p[0]; // >= 0
    res[2] = 1.0*( QDOT[0] * QDOT[0] ) - p[0]; // >= 0
    res[3] = 1.0*( QDOT[1] * QDOT[1] ) - p[0]; // >= 0
}

// ********************************o
// *
// * Coupled Constraints
// *
// ********************************o
static int RCFCN_N = 0, RCFCN_NE = 0;
static void rcfcn_s (double *ts, double *sd, double *sa, double *u, double *p, double *pr, double *res, long *dpnd, InfoPtr *info) {
    if (*dpnd) {
        *dpnd = RFCN_DPND(0, *sd, *u, 0, 0, 0);
        return;
    }
}

static void rcfcn_e (double *ts, double *sd, double *sa, double *u, double *p, double *pr, double *res, long *dpnd, InfoPtr *info) {
    if (*dpnd) {
        *dpnd = RFCN_DPND(0, *sd, *sa, *u, *p, 0);
        return;
    }
}

// ********************************o
// *
// * Data Output
// *
// ********************************o

vector<double> t_values;
vector<vector<double> > sd_values;
vector<vector<double> > u_values;

static void data_out( double *t, double *sd, double *sa, double *u, double *p, double *rwh, long *iwh, InfoPtr *info ) {
    if (*t == 0.) {
        ofstream meshup_csv_stream;

        string meshup_header_file = string(rel_data_path) + string("RES/meshup_") + string(problem_name) + string(".csv");
        string data_sd_file       = string(rel_data_path) + string("sd_")     + string(problem_name) + string(".csv");
        string data_u_file        = string(rel_data_path) + string("u_")      + string(problem_name) + string(".csv");

        ofstream meshup_header_stream;
        ofstream data_sd_stream;
        ofstream data_u_stream;

        meshup_header_stream.open (meshup_header_file.c_str(), ios_base::trunc);
        data_sd_stream.open (data_sd_file.c_str(), ios_base::trunc);
        data_u_stream.open (data_u_file. c_str(), ios_base::trunc);

        if (!meshup_header_stream || !data_sd_stream || !data_u_stream) {
            cerr << "Error opening file (" << meshup_header_file << ") or ("
                 << data_sd_file << ") or (" << data_u_file << ")" << endl;
            abort();
        }
        const char* meshup_header = "COLUMNS: \n\
    time,\n\
    cart:T:x,\n\
    pendulum:R:y:rad,\
";
        meshup_header_stream << meshup_header << endl << "DATA_FROM: " << data_sd_file << endl;

        if (t_values.size() > 0) {
            for (unsigned int i = 0; i < t_values.size(); i ++) {
                data_sd_stream << t_values[i] << ", ";
                for (unsigned int j = 0; j < sd_values[i].size(); j++) {
                    data_sd_stream << sd_values[i][j];
                    if (j < sd_values[i].size() -1 )
                        data_sd_stream << ", ";
    }
                data_sd_stream << endl;

                data_u_stream << t_values[i] << ", ";
                for (unsigned int j = 0; j < u_values[i].size(); j++) {
                    data_u_stream << u_values[i][j];
                    if (j < u_values[i].size() -1 )
                        data_u_stream << ", ";
                }
                data_u_stream << endl;
        }
    }

        t_values.clear();
        sd_values.clear();
        u_values.clear();

        meshup_header_stream.close();
        data_sd_stream.close();
        data_u_stream.close();
    }

    t_values.push_back (*t);

    vector<double> sd_vec (NXD);
    for (unsigned i = 0; i < NXD; i++)
        sd_vec[i] = sd[i];
    sd_values.push_back (sd_vec);

    vector<double> u_vec (NU);
    for (unsigned i = 0; i < NU; i++)
        u_vec[i] = u[i];
    u_values.push_back (u_vec);
}

void meshup_output
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
)
{
    InfoPtr info(0, *imos, *imsn);
    data_out( ts, sd, sa, u, p, rwh, iwh, &info);
}

/*
LuaTable readLuaModel(const char *filename)
{
  lua_State *L = luaL_newstate();
  luaL_openlibs(L);
  luaL_dofile (L, filename);
  return LuaTable::fromLuaState(L);
}*/

// \brief Entry point for the muscod application
extern "C" void def_model(void);
void def_model(void)
{
    memset (problem_name, 0, 255);
    get_pname (problem_name);

    // open and process reference trajectories
    xref_setup();

    // load LUA model
    model = new Model;
    std::string model_file_name_full = rel_data_path + model_file_name; // add relative path
    cout << "Loading RBDL model from: '" << model_file_name_full << "'" << endl;
//    if (!RigidBodyDynamics::Addons::LuaModelReadFromTable(readLuaModel(model_file_name_full.c_str()), model, false))
    if (!Addons::LuaModelReadFromFile (model_file_name_full.c_str(), model, false))
    {
        cerr << "Error loading RBDL model '" << model_file_name_full << "'!" << endl;
        abort();
    }

    assert (model->dof_count == 2);

    nDof = model->dof_count;
    nActuatedDof = 1;

    Q     = VectorNd::Zero(model->dof_count);
    QDOT  = VectorNd::Zero(model->dof_count);
    QDDOT = VectorNd::Zero(model->dof_count);
    TAU   = VectorNd::Zero(model->dof_count);

    // define problem dimensions
    NMOS = 1;
    NP   = 1;
    NRC  = 0;
    NRCE = 0;

    NXD    = nDof*2;
    NXA    = 0;
    NU     = nActuatedDof;
    NPR    = 0;

    def_mdims(NMOS, NP, NRC, NRCE);

    // right_flat
    def_mstage(
            0,
            NXD, NXA, NU,
            NULL,         NULL,
            0, 0, 0, NULL, ffcn, NULL,
            NULL, NULL
            );

    // define LSQ objective
    def_lsq(0, "*", 0, 4, lsqfcn);

    def_mio (NULL , meshup_output, data_out);

#ifdef HAVE_USERPLOT
    def_userplot (plot_start, plot_data, plot_end);
#endif
}
