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

#include <def_usrmod.hpp>
#include <model.hpp>
#include <mhe_cartpole.h>

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
namespace CommonMHE { // BEGIN NAMESPACE CommonMHE
// *****************************************************************************

// *****************************************************************************
// Model
// *****************************************************************************

// RBDL Models for sequential or parallel execution
RBDLModel rbdl_model;
std::vector<RBDLModel> rbdl_models;

// *****************************************************************************
// Constants
// *****************************************************************************

// RL discount factor
const double rl_y = 0.9900;
const double rl_sy = sqrt(rl_y);

// weights
const double w0 =  1.0000;
const double w1 = 10.0000;
const double w2 =  0.1000;
const double w3 =  0.1000;
const double w4 =  0.0001;

const double rl_w0 = 2.0000;
const double rl_w1 = 1.0000;
const double rl_w2 = 0.2000;
const double rl_w3 = 0.5000;
// const double rl_w4 = 0.0001;
const double rl_w4 = 0.0005;
// const double rl_w4 = 0.0010;

// square-roots of weights for Gauss-Newton residual
const double sw0 = sqrt(w0);
const double sw1 = sqrt(w1);
const double sw2 = sqrt(w2);
const double sw3 = sqrt(w3);
const double sw4 = sqrt(w4);

const double rl_sw0 = sqrt(rl_w0);
const double rl_sw1 = sqrt(rl_w1);
const double rl_sw2 = sqrt(rl_w2);
const double rl_sw3 = sqrt(rl_w3);
const double rl_sw4 = sqrt(rl_w4);

std::string model_file_name = (std::string) "./model.lua";
const std::string ref_states_path = "RES/sd_cart_pendulum_time.csv";
const std::string ref_cntrls_path = "RES/u_cart_pendulum_time.csv";

// *****************************************************************************
// Variables
// *****************************************************************************

bool is_problem_name_initialized = false;
bool is_tikhonov_initialized = false;
bool is_horizon_initialized = false;
bool add_counter = false;
char problem_name[255];
long real_nmos, real_nmsn;

std::ifstream ref_states_f;
std::ifstream ref_cntrls_f;

std::string rel_data_path = "";

  // reference trajectories for NMPC
std::vector<double> ref_time;
std::vector<std::vector<double> > ref_states;
std::vector<std::vector<double> > ref_cntrls;

// measurements for MHE
std::vector<double> measurement_times;
std::vector<std::vector<double> > measurements;
std::vector<std::vector<double> > sigmas;

// placeholder for plotting
std::vector<double> _plotting_t_values;
std::vector<double> _plotting_p_values;
std::vector<std::vector<double> > _plotting_sd_values;
std::vector<std::vector<double> > _plotting_u_values;

std::vector<double> _plotting_ts_values;
std::vector<std::vector<double> > _plotting_hs_values;
std::vector<std::vector<double> > _plotting_ss_values;

// Tikhonov regularization
Eigen::VectorXd p0(NP);
Eigen::VectorXd s0(NP);

// measurement horizon
// Eigen::VectorXd m_ts; // measurement times
Eigen::MatrixXd meas_hs; // measurements
Eigen::MatrixXd meas_ss; // measurement errors

// *****************************************************************************
// Utilities
// *****************************************************************************
extern "C" {
  void set_path(std::string new_problem_path, std::string new_lua_model){
    rel_data_path = new_problem_path + "/";
    model_file_name = new_lua_model;
    std::cout << "MHE: setting new problem path to: '" << rel_data_path << "'" <<std::endl;
    std::cout << "MHE: setting new Lua model file to: '" << model_file_name << "'" <<std::endl;
  }

  bool get_plot_counter(){
    return add_counter;
  }

  void set_plot_counter(bool counter){
    add_counter = counter;
    std::cout << "plot counter is set to: " << get_plot_counter() << std::endl;
  }

    void print_horizon(){
      // std::cout << "meas_ts = " << meas_ts.transpose() << std::endl;
      // std::cout << std::endl;

      std::cout << "meas_hs = " << std::endl << meas_hs << std::endl;
      std::cout << std::endl;

      std::cout << "meas_ss = " << std::endl << meas_ss << std::endl;
      std::cout << std::endl;
    }

    void inject_measurement(
      // double& ts,
      Eigen::VectorXd& hs,
      Eigen::VectorXd& ss
    ){
      // first get measurements
      unsigned int n = meas_hs.cols() - 1;
      // m_ts.head(n) = m_ts.tail(n);
      meas_hs.leftCols(n) = meas_hs.rightCols(n);
      meas_ss.leftCols(n) = meas_ss.rightCols(n);

      // second inject ts, hs and ss
      // m_ts.tail(1) = ts;
      meas_hs.rightCols(1) = hs;
      meas_ss.rightCols(1) = ss;
    }

    void get_reference_to_horizon(
      // Eigen::VectorXd m_ts,
      Eigen::MatrixXd* hs,
      Eigen::MatrixXd* ss
    ) {
      // assign pointers
      hs = &meas_hs;
      ss = &meas_ss;

      std::cout << "received reference to measurement horizon" << std::endl;
  }

  Eigen::MatrixXd* get_measurements()
  {
    return &meas_hs;
  }

  Eigen::MatrixXd* get_sigmas()
  {
    return &meas_ss;
  }

} // END of extern "C"

  // Convenience function to check consistency of variable dimensions
void check_dimensions(
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

// *****************************************************************************

  void load_from_file(
    std::vector<double>& time, std::vector<std::vector<double> >& values,
    const std::string path, const std::string rel_path
    ){
  // place holder for line string and file handle
    std::string line;
    std::ifstream file;

  // add a relative path
    std::string path_full = rel_path + path;

  // load file
    file.open(path_full.c_str());

    // clear data if not empty
    if (time.empty()) {
      time.clear();
    }
    if (values.empty()) {
      values.clear();
    }

  // process file
    if (file.is_open()) {
      while (std::getline(file, line)) {
      // Insert the string into a stream
        std::istringstream ss(line);

      // process entries per line
        double dummy;
        std::vector<double> entries;

        while(ss >> dummy){
          entries.push_back(dummy);

          // state exceptions to ignore
          // NOTE: this will separate the entires of the CSV file
          if (ss.peek() == ',' || ss.peek() == ' ') {
            ss.ignore();
          }
        }

      // store entries in appropriate container
      // NOTE: first entry is always time, rest are values
      time.push_back(entries[0]); // extract time
      entries.erase(entries.begin()); // remove first entry

      values.push_back(entries);
    }
  } else {
    std::cerr << "ERROR: Could not open file: " << path_full << std::endl;
    std::cerr << "bailing out ..." << std::endl;
    exit(-1);
  }

  // close file when finished
  file.close();
}

// *****************************************************************************
void ref_setup(
  const std::string ref_states_path, const std::string ref_cntrls_path
  ){
  std::string line;
  std::ifstream ref_states_f;
  std::ifstream ref_cntrls_f;

  // add a relative path
  std::string ref_states_path_full = rel_data_path + ref_states_path;
  std::string ref_cntrls_path_full = rel_data_path + ref_cntrls_path;

  // load file
  ref_states_f.open(ref_states_path.c_str());

      // process file
  if (ref_states_f.is_open()) {
    while (std::getline(ref_states_f, line)) {
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
      ref_time.push_back(values[0]);
          values.erase(values.begin()); // remove first entry

          ref_states.push_back(values);
        }
      } else {
        std::cerr << "ERROR: Could not open file: " << ref_states_path << std::endl;
        exit(-1);
      }
      ref_states_f.close();

      // load file
      ref_cntrls_f.open(ref_cntrls_path.c_str());
      // process file
      if (ref_cntrls_f.is_open()) {
        while (std::getline(ref_cntrls_f, line)) {
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
          ref_cntrls.push_back(values);
        }
      } else {
        std::cerr << "ERROR: Could not open file: " << ref_states_path << std::endl;
        exit(-1);
      }
      ref_cntrls_f.close();
    }

// *****************************************************************************

    void closest(const std::vector<double>& vec, const double& value, int& index) {
      std::vector<double>::const_iterator it;
      it = std::lower_bound(vec.begin(), vec.end(), value);
      if (it == vec.end()) {
        index = -1;
      } else {
        index = std::distance(vec.begin(), it);
      }
    }

// *****************************************************************************

    double get_value_from_data(
      const std::vector<double> times,
      const std::vector<std::vector<double> > data,
      const int choice, const double t
      ) {
      // placeholder for index of data entry
      int index;

      closest(times, t, index);

      if(index<0){
        return 0.0;
      } else {
        return data[index][choice];
      }
    }

// *****************************************************************************

  // double (*placeholder) (
  //     const std::vector<double> time = std::vector<double> (),
  //     const std::vector<std::vector<double>> data,
  //     const int choice, const double t
  // ) {
  //   return
  // }

// *****************************************************************************
// reference trajectories for NMPC
// *****************************************************************************


    double ref_xd(const int choice, const double t){
      int index;
    // find closest
      closest(ref_time, t, index);
      if(index<0){
        return 0.0;
      } else {
        std::vector<double> state = ref_states[index];
        return state[choice];
      }
    }

    double ref_u(const int choice, const double t){
      int index;
    // find closest
      closest(ref_time, t, index);
      if(index<0){
        return 0.0;
      } else {
        std::vector<double> cntrl = ref_cntrls[index];
        return cntrl[choice];
      }
    }


// *****************************************************************************
// Objective Functions (Lagrange Type)
// *****************************************************************************

    void lfcn_time(
      double *t, double *xd, double *xa, double *u, double *p, double *lval,
      double *rwh, long *iwh, InfoPtr *info
      ){
    // zeroize Lagrange
      *lval = 0.0;

    // define objective
    *lval = 1.0; // minimize time by int_0^T 1 dt = T
  }

  // Objective function (Lagrangian type)
  void lfcn_energy(
    double *t, double *xd, double *xa, double *u, double *p, double *lval,
    double *rwh, long *iwh, InfoPtr *info
    ){
    // zeroize Lagrange
    *lval = 0.0;

    // get right RBDL model
    RBDLModel* model;
    model = &rbdl_models[info->cnode];

    // update_states
    model->update_state(xd, u, p);

    // minimize energy by means of L2-norm of actuation
    for (int idof = 0; idof < model->nadof; idof++) {
      *lval += model->tau[idof] * model->tau[idof];
    }
  }

  void lfcn_tracking(
    double *t, double *xd, double *xa, double *u,
    double *p, double *lval, double *rwh, long *iwh, InfoPtr *info
    ) {

  // integrate over constant
    *lval = 0.0;

  // define objective
  *lval += w0 * ( xd[0] ) * ( xd[0] ); // = 0
  *lval += w1 * ( xd[1] ) * ( xd[1] ); // = 0
  *lval += w2 * ( xd[2] ) * ( xd[2] ); // = 0
  *lval += w3 * ( xd[3] ) * ( xd[3] ); // = 0
  *lval += w4 * (  u[0] ) * (  u[0] ); // = 0
}

// ****************************************************************************
// Objectives Mayer Type
// ****************************************************************************

void mfcn_end_time(
  double *ts, double *xd, double *xa, double *p, double *pr, double *mval,
  long *dpnd, InfoPtr *info
  ){
    // define dependencies
    // NOTE: Dependency pattern determines the derivatives to be computed!
  if (*dpnd) {
      // choose dependency pattern
    bool ts_dpnd = true;
    bool xd_dpnd = false;
    bool xa_dpnd = false;
    bool  p_dpnd = false;
    bool pr_dpnd = false;
      // automatically resolve dependencies
    *dpnd = MFCN_DPND(
      (double) (ts_dpnd?*ts:0), (double) (xd_dpnd?*xd:0),
      (double) (xd_dpnd?*xa:0), (double) (p_dpnd? *p:0),
      (double) (pr_dpnd?*pr:0)
      );
    return;
  }
    // zeroize Mayer
  *mval = 0.0;

    // define objective
    *mval = *ts;  // minimize end time
  }

/** \brief Objective function (Mayer type) */
  void mfcn_penalty(
    double *ts, double *xd, double *xa, double *p, double *pr,
    double *mval,  long *dpnd, InfoPtr *info
    ) {
    // define dependencies
    // NOTE: Dependency pattern determines the derivatives to be computed!
    if (*dpnd) {
      // choose dependency pattern
      bool ts_dpnd = false;
      bool xd_dpnd = true;
      bool xa_dpnd = false;
      bool  p_dpnd = false;
      bool pr_dpnd = false;
      // automatically resolve dependencies
      *dpnd = MFCN_DPND(
        (double) (ts_dpnd?*ts:0), (double) (xd_dpnd?*xd:0),
        (double) (xd_dpnd?*xa:0), (double) (p_dpnd? *p:0),
        (double) (pr_dpnd?*pr:0)
        );
      return;
    }

  // zeroize Mayer
    *mval = 0.0;

  *mval +=  w0 * ( xd[0] ) * ( xd[0] ); // = 0
  *mval +=  w1 * ( xd[1] ) * ( xd[1] ); // = 0
  *mval +=  w2 * ( xd[2] ) * ( xd[2] ); // = 0
  *mval +=  w3 * ( xd[3] ) * ( xd[3] ); // = 0
}

// *****************************************************************************
// Least Squares Functions
// *****************************************************************************

const unsigned int LSQFCN_FIT_NE = 4;
void lsqfcn_fit(double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
  if (*dpnd) { *dpnd = RFCN_DPND(0, *sd, 0, 0, 0, 0); return; }

  // get measurements
  const double h_0 = get_value_from_data(measurement_times, measurements, 0, *ts);
  const double h_1 = get_value_from_data(measurement_times, measurements, 1, *ts);
  const double h_2 = get_value_from_data(measurement_times, measurements, 2, *ts);
  const double h_3 = get_value_from_data(measurement_times, measurements, 3, *ts);

  // get sigmas
  const double s_0 = 1.0; // get_value_from_data(measurement_times, sigmas, 0, info->cnode)
  const double s_1 = 1.0; // get_value_from_data(measurement_times, sigmas, 1, info->cnode)
  const double s_2 = 1.0; // get_value_from_data(measurement_times, sigmas, 2, info->cnode)
  const double s_3 = 1.0; // get_value_from_data(measurement_times, sigmas, 3, info->cnode)

  // L   = ||r(x)||_2^2
  // res = r(t,x,u,p)
  //     = (h - h_) / s
  res[0] = (sd[0] - h_0) / s_0;
  res[1] = (sd[1] - h_1) / s_1;
  res[2] = (sd[2] - h_2) / s_2;
  res[3] = (sd[3] - h_3) / s_3;
}

const unsigned int LSQFCN_FITTING_NE = 4;
void lsqfcn_fitting(double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
  // define dependencies
  // NOTE: Dependency pattern determines the derivatives to be computed!
  if (*dpnd) {
    // choose dependency pattern
    bool ts_dpnd = false;
    bool xd_dpnd = true;
    bool xa_dpnd = false;
    bool  u_dpnd = false;
    bool  p_dpnd = false;
    bool pr_dpnd = false;
    // automatically resolve dependencies
    *dpnd = RFCN_DPND(
      (double) (ts_dpnd?*ts:0), (double) (xd_dpnd?*sd:0),
      (double) (xa_dpnd?*sa:0), (double) (u_dpnd?*u:0),
      (double) (p_dpnd? *p:0), (double) (pr_dpnd?*pr:0)
      );
    return;
  }

  // get measurements and errors
  if (!is_horizon_initialized) {
    meas_hs = Eigen::MatrixXd::Zero(NXD + NU, real_nmsn);
    meas_ss = Eigen::MatrixXd::Ones(NXD + NU, real_nmsn);
    is_horizon_initialized = true;
  }
  Eigen::VectorXd hs = meas_hs.col(info->cnode);
  Eigen::VectorXd ss = meas_ss.col(info->cnode);

  // L   = ||r(x)||_2^2
  // res = r(t,x,u,p)
  //     = (h - h_) / s
  res[0] = (sd[0] - hs(0)) / ss(0);
  res[1] = (sd[1] - hs(1)) / ss(1);
  res[2] = (sd[2] - hs(2)) / ss(2);
  res[3] = (sd[3] - hs(3)) / ss(3);
}

const unsigned int LSQFCN_FITTING_REG_NE = 5 + 1;
void lsqfcn_fitting_reg(
  double *ts, double *sd, double *sa, double *u, double *p, double *pr,
  double *res, long *dpnd, InfoPtr *info
) {
  // define dependencies
  // NOTE: Dependency pattern determines the derivatives to be computed!
  if (*dpnd) {
    // choose dependency pattern
    bool ts_dpnd = false;
    bool xd_dpnd = true;
    bool xa_dpnd = false;
    bool  u_dpnd = true;
    bool  p_dpnd = true;
    bool pr_dpnd = false;
    // automatically resolve dependencies
    *dpnd = RFCN_DPND(
      (double) (ts_dpnd?*ts:0), (double) (xd_dpnd?*sd:0),
      (double) (xd_dpnd?*sa:0), (double) (u_dpnd?*u:0),
      (double) (p_dpnd? *p:0), (double) (pr_dpnd?*pr:0)
      );
    return;
  }
  // define constraint counters
  unsigned int res_ne_cnt = 0;

  // debug output
  // std::cout << "cnode = " << info->cnode << ",\t";
  // std::cout << "time =  " << *ts << ", ";
  // std::cout << std::endl;

  // get measurements and errors
  // NOTE: lazy initialization because MUSCOD sucks
  if (!is_horizon_initialized) {
    // std::cout << "real_nmsn = " << real_nmsn << std::endl;
    meas_hs = Eigen::MatrixXd::Zero(NXD + NU, real_nmsn);
    meas_ss = Eigen::MatrixXd::Ones(NXD + NU, real_nmsn);
    is_horizon_initialized = true;
  }
  Eigen::VectorXd hs = meas_hs.col(info->cnode);
  Eigen::VectorXd ss = meas_ss.col(info->cnode);

  // L   = ||r(x)||_2^2
  // res = r(t,x,u,p)
  //     = (h - h_) / s
  res[res_ne_cnt++] = (sd[0] - hs(0)) / ss(0);
  res[res_ne_cnt++] = (sd[1] - hs(1)) / ss(1);
  res[res_ne_cnt++] = (sd[2] - hs(2)) / ss(2);
  res[res_ne_cnt++] = (sd[3] - hs(3)) / ss(3);

  res[res_ne_cnt++] = ( u[0] - hs(4)) / ss(4);

  // Tikhonov regularization
  // || (p - p0) / s0 ||
  if (!is_tikhonov_initialized) {
    p0 << 0.0;
    s0 << 1000.0;
  }
  res[res_ne_cnt++] = (p[0] - p0[0]) / s0[0];

  // check dimensions of function
  check_dimensions(0, 0, res_ne_cnt, LSQFCN_FITTING_REG_NE, __func__);
}

const unsigned int LSQFCN_TRACKING_REF_NE = 4;
void lsqfcn_tracking_ref(
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
      (double) (ts_dpnd?*ts:0), (double) (sd_dpnd?*sd:0),
      (double) (sa_dpnd?*sa:0), (double) (u_dpnd?*u:0),
      (double) (p_dpnd? *p:0),  (double) (pr_dpnd?*pr:0)
      );
    return;
  }
    // define constraint counters
  unsigned int res_ne_cnt = 0;

    // define constraint residuals here, i.e. res[:_ne] = 0 and res[_ne:] >= 0!
    // NOTE: In C++ the '++' operator as postfix first evaluates the variable
    //       and then increments it, i.e.
    //       n = 0;    -> n  = 0
    //       n2 = n++; -> n2 = 0, n = 1

    // res = r(t,x,u,p)
    // L   = ||r(x)||_2^2

    // get real_time from nmpc_loop
  double real_ts = *ts + p[0];

    // res = sqrt(w) * quantity
  res[res_ne_cnt++] = sw0 * ( sd[0] - ref_xd(0, real_ts) );
  res[res_ne_cnt++] = sw1 * ( sd[1] - ref_xd(1, real_ts) );
  res[res_ne_cnt++] = sw2 * ( sd[2] - ref_xd(2, real_ts) );
  res[res_ne_cnt++] = sw3 * ( sd[3] - ref_xd(3, real_ts) );

    // check dimensions of function
  check_dimensions(0, 0, res_ne_cnt, LSQFCN_TRACKING_REF_NE, __func__);
}

const unsigned int LSQFCN_TRACKING_NE = 5;
void lsqfcn_tracking(
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
    bool  p_dpnd = false;
    bool pr_dpnd = false;
      // automatically resolve dependencies
    *dpnd = RFCN_DPND(
      (double) (ts_dpnd?*ts:0), (double) (sd_dpnd?*sd:0),
      (double) (sa_dpnd?*sa:0), (double) (u_dpnd?*u:0),
      (double) (p_dpnd? *p:0),  (double) (pr_dpnd?*pr:0)
      );
    return;
  }
    // define constraint counters
  unsigned int res_ne_cnt = 0;

    // define constraint residuals here, i.e. res[:_ne] = 0 and res[_ne:] >= 0!
    // NOTE: In C++ the '++' operator as postfix first evaluates the variable
    //       and then increments it, i.e.
    //       n = 0;    -> n  = 0
    //       n2 = n++; -> n2 = 0, n = 1

    // L = ||r(t,x,u,p)||_2^2
    // r(t,x,u,p) = sqrt(w) * quantity
  double discount = pow(rl_y, double(info->cnode)/2.);
  // std::cout << "cnode    = " << info->cnode << std::endl;
  // std::cout << "discount = " << discount << std::endl;

  res[res_ne_cnt++] = rl_sw0 * sd[0] * discount;
  res[res_ne_cnt++] = rl_sw1 * sd[1] * discount;
  res[res_ne_cnt++] = rl_sw2 * sd[2] * discount;
  res[res_ne_cnt++] = rl_sw3 * sd[3] * discount;
  res[res_ne_cnt++] = rl_sw4 * u[0]  * discount;

    // check dimensions of function
  check_dimensions(0, 0, res_ne_cnt, LSQFCN_TRACKING_NE, __func__);
}

// *****************************************************************************
// Right Hand Sides
// *****************************************************************************

void ffcn_st (
  double *t, double *xd, double *xa, double *u, double *p, double *rhs,
  double *rwh, long *iwh, InfoPtr *info
  ) {
    // define rhs counter
  unsigned int rhs_cnt = 0;

  // update_states
  rbdl_model.update_state(xd, u, p);

  // calculate forward dynamics using RBDL
  RigidBodyDynamics::ForwardDynamics (
    rbdl_model.model, rbdl_model.q, rbdl_model.qdot, rbdl_model.tau, rbdl_model.qddot
  );

  for (unsigned int idof = 0; idof < rbdl_model.ndof; idof++) {
    rhs[idof] = rbdl_model.qdot [idof]; rhs_cnt++;
    rhs[idof + rbdl_model.ndof] = rbdl_model.qddot[idof]; rhs_cnt++;
  }

    // check dimensions of functions
  check_dimensions(0, 0, rhs_cnt, 4, __func__);
}

void ffcn (
  double *t, double *xd, double *xa, double *u, double *p, double *rhs,
  double *rwh, long *iwh, InfoPtr *info
  ) {
    // define rhs counter
  unsigned int rhs_cnt = 0;

  // get right RBDL model
  RBDLModel* model;
  model = &rbdl_models[info->cnode];

  // update_states
  model->update_state(xd, u, p);

  // calculate forward dynamics using RBDL
  RigidBodyDynamics::ForwardDynamics (
    model->model, model->q, model->qdot, model->tau, model->qddot
  );

  for (unsigned int idof = 0; idof < model->ndof; idof++) {
    rhs[idof] = model->qdot [idof]; rhs_cnt++;
    rhs[idof + model->ndof] = model->qddot[idof]; rhs_cnt++;
  }

    // check dimensions of functions
  check_dimensions(0, 0, rhs_cnt, 4, __func__);
}

// *****************************************************************************
// Decoupled Constraints
// *****************************************************************************

///  Constraints at start point
//                 # of all constraints # of equality constraints
const unsigned int RDFCN_S_N = 4,       RDFCN_S_NE = 4;
void rdfcn_s(
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
      (double) (ts_dpnd?*ts:0), (double) (sd_dpnd?*sd:0),
      (double) (sa_dpnd?*sa:0), (double) (u_dpnd?*u:0),
      (double) (p_dpnd? *p:0),  (double) (pr_dpnd?*pr:0)
      );
    return;
  }
  // define constraint counters
  unsigned int res_n_cnt  = 0;
  unsigned int res_ne_cnt = 0;

  // define constraint residuals here, i.e. res[:_ne] = 0 and res[_ne:] >= 0!
  // NOTE: In C++ the '++' operator as postfix first evaluates the variable
  //       and then increments it, i.e.
  //       n = 0;    -> n  = 0
  //       n2 = n++; -> n2 = 0, n = 1

    // get right RBDL model
    RBDLModel* model;
    model = &rbdl_models[info->cnode];

    // update_states
  model->update_state(sd, u, p);

  // enforce pendulum hanging down on start position
  // NOTE: We have to scale residuals to enforce them in the solution
  res[res_n_cnt++] = (        model->q[0] ) * 100.0; res_ne_cnt++;// = 0
  res[res_n_cnt++] = ( 3.14 - model->q[1] ) * 100.0; res_ne_cnt++;// = 0
  res[res_n_cnt++] = (     model->qdot[0] ) * 100.0; res_ne_cnt++;// = 0
  res[res_n_cnt++] = (     model->qdot[1] ) * 100.0; res_ne_cnt++;// = 0

  // check dimensions of constraints
  check_dimensions(
    res_n_cnt, RDFCN_S_N, res_ne_cnt, RDFCN_S_NE, __func__
    );
}

///  Constraints at end point
//                 # of all constraints # of equality constraints
const unsigned int RDFCN_E_N = 4,       RDFCN_E_NE = 4;
void rdfcn_e(
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
      (double) (ts_dpnd?*ts:0), (double) (sd_dpnd?*sd:0),
      (double) (sa_dpnd?*sa:0), (double) (u_dpnd?*u:0),
      (double) (p_dpnd? *p:0),  (double) (pr_dpnd?*pr:0)
      );
    return;
  }
  // define constraint counters
  unsigned int res_n_cnt  = 0;
  unsigned int res_ne_cnt = 0;

  // define constraint residuals here, i.e. res[:_ne] = 0 and res[_ne:] >= 0!
  // NOTE: In C++ the '++' operator as postfix first evaluates the variable
  //       and then increments it, i.e.
  //       n = 0;    -> n  = 0
  //       n2 = n++; -> n2 = 0, n = 1

  // get right RBDL model
  RBDLModel* model;
  model = &rbdl_models[info->cnode];

    // update_states
    model->update_state(sd, u, p);

  // enforce pendulum being upright in final position
  // NOTE: We have to scale residuals to enforce them in the solution
  res[res_n_cnt++] = ( model->q[0] ) * 100.0; res_ne_cnt++;// = 0
  res[res_n_cnt++] = ( model->q[1] ) * 100.0; res_ne_cnt++;// = 0
  res[res_n_cnt++] = ( model->qdot[0] ) * 100.0; res_ne_cnt++;// = 0
  res[res_n_cnt++] = ( model->qdot[1] ) * 100.0; res_ne_cnt++;// = 0

  // check dimensions of constraints
  check_dimensions(
    res_n_cnt, RDFCN_E_N, res_ne_cnt, RDFCN_E_NE, __func__
    );
}

// *****************************************************************************
// Data Output
// *****************************************************************************

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
  // count model stages and shooting nodes
  real_nmos = std::max(real_nmos, *imos);
  real_nmsn = std::max(real_nmsn, *imsn);

  // setup RBDL models for each shooting node! for each shooting node!
  RBDLModel rbdl_model;
  rbdl_model.load_from_file(model_file_name.c_str());
  rbdl_models.push_back(rbdl_model);
}


void data_out(
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

    std::string data_hs_file        = std::string("hs_");
    std::string data_ss_file        = std::string("ss_");

    // add problem name identifier
    meshup_header_file += std::string(problem_name);
    data_sd_file       += std::string(problem_name);
    data_u_file        += std::string(problem_name);
    data_p_file        += std::string(problem_name);

    data_hs_file       += std::string(problem_name);
    data_ss_file       += std::string(problem_name);

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

      data_hs_file       += sstm.str();
      data_ss_file       += sstm.str();
    }

    // add file identifier suffix
    meshup_header_file += std::string(".csv");
    data_sd_file       += std::string(".csv");
    data_u_file        += std::string(".csv");
    data_p_file        += std::string(".csv");

    data_hs_file        += std::string(".csv");
    data_ss_file        += std::string(".csv");

    std::ofstream meshup_header_stream;
    std::ofstream data_sd_stream;
    std::ofstream data_u_stream;
    std::ofstream data_p_stream;

    std::ofstream data_hs_stream;
    std::ofstream data_ss_stream;

    meshup_header_stream.open (meshup_header_file.c_str(),                   std::ios_base::trunc);
    data_sd_stream.      open ((std::string("RES/") + data_sd_file).c_str(), std::ios_base::trunc);
    data_u_stream.       open ((std::string("RES/") + data_u_file). c_str(), std::ios_base::trunc);
    data_p_stream.       open ((std::string("RES/") + data_p_file). c_str(), std::ios_base::trunc);

    data_hs_stream.      open ((std::string("RES/") + data_hs_file). c_str(), std::ios_base::trunc);
    data_ss_stream.      open ((std::string("RES/") + data_ss_file). c_str(), std::ios_base::trunc);

    if (
      !meshup_header_stream || !data_sd_stream || !data_u_stream ||
      !data_p_stream || !data_hs_stream || !data_ss_stream
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
      if (!data_hs_stream) {
        std::cerr << data_hs_file;
      }
      if (!data_ss_stream) {
        std::cerr << data_ss_file;
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

      // save shooting node dependent quantities
      for (unsigned int i = 0; i < _plotting_ts_values.size() - 1; i ++) {
        // states
        data_hs_stream << _plotting_ts_values[i] << ", ";
        for (unsigned int j = 0; j < _plotting_hs_values[i].size(); j++) {
          data_hs_stream << _plotting_hs_values[i][j];
          if (j < _plotting_hs_values[i].size() -1 )
            data_hs_stream << ", ";
        }
        data_hs_stream << std::endl;

        // controls
        data_ss_stream << _plotting_ts_values[i] << ", ";
        for (unsigned int j = 0; j < _plotting_ss_values[i].size(); j++) {
          data_ss_stream << _plotting_ss_values[i][j];
          if (j < _plotting_ss_values[i].size() -1 )
            data_ss_stream << ", ";
        }
        data_ss_stream << std::endl;
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

    _plotting_ts_values.clear();
    _plotting_hs_values.clear();
    _plotting_ss_values.clear();

    meshup_header_stream.close();
    data_sd_stream.close();
    data_u_stream.close();
    data_p_stream.close();

    data_hs_stream.close();
    data_ss_stream.close();

    lnode = -1;
  }

  if (info->cnode != lnode) {
    if (info->cnode == 0) {
      for (unsigned i = 0; i < NP; i++) {
        _plotting_p_values.push_back(p[i]);
      }
    }
    std::cout << "t = " << *t << "\t" << "cnode = " << info->cnode << std::endl;

    if (info->cnode < meas_hs.cols() && info->cnode < meas_ss.cols()) {
      _plotting_ts_values.push_back (*t);

      std::vector<double> _hs_vec (NXD + NU);
      for (unsigned i = 0; i < NXD + NU; i++){
        _hs_vec[i] = meas_hs(i, info->cnode);
      }
      _plotting_hs_values.push_back (_hs_vec);

      std::vector<double> _ss_vec (NXD + NU);
      for (unsigned i = 0; i < NXD + NU; i++){
        _ss_vec[i] = meas_ss(i, info->cnode);
      }
      _plotting_ss_values.push_back (_ss_vec);
    }

    // pass new node value
    lnode = info->cnode;
  }

  _plotting_t_values.push_back (*t);

  std::vector<double> sd_vec (4);
  for (unsigned i = 0; i < 4; i++)
    sd_vec[i] = sd[i];
  _plotting_sd_values.push_back (sd_vec);

  std::vector<double> u_vec (1);
  for (unsigned i = 0; i < 1; i++)
    u_vec[i] = u[i];
  _plotting_u_values.push_back (u_vec);
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
    ) {
  InfoPtr info(0, *imos, *imsn);
  data_out( ts, sd, sa, u, p, rwh, iwh, &info);
}

// *****************************************************************************
} // END NAMESPACE CommonMHE
// *****************************************************************************

// *****************************************************************************
// Constants
// *****************************************************************************

// define MUSCOD-II Dimensions
// NOTE: To resolve ambiguity we explicitly use CommonMHE::
const unsigned int CommonMHE::NMOS = 1;  // Number of phases (MOdel Stages)
const unsigned int CommonMHE::NP   = 1;  // Number of parameters
const unsigned int CommonMHE::NRC  = 0;  // Number of coupled constraints
const unsigned int CommonMHE::NRCE = 0;  // Number of coupled equality constraints

const unsigned int CommonMHE::NXD  = 4;  // Number of differential states
const unsigned int CommonMHE::NXA  = 0;  // Number of algebraic states
const unsigned int CommonMHE::NU   = 1;  // Number of controls
const unsigned int CommonMHE::NPR  = 0;  // Number of local parameters

// *****************************************************************************
// MUSCOD Application
// *****************************************************************************

// Entry point for the muscod application
extern "C" void def_model(void);
void def_model(void)
{
    // *********************************
    // Model Stage: Swing-Up and Balance
    // *********************************
    unsigned long imos = 0;

    // set model dimensions
    def_mdims(
        CommonMHE::NMOS, CommonMHE::NP, CommonMHE::NRC, CommonMHE::NRCE
    );

    // right_flat
    def_mstage(
        imos, // imos,
        // nxd, nxa, nu,
        CommonMHE::NXD, CommonMHE::NXA, CommonMHE::NU,
        NULL, // MayPtr mfcn
        NULL, // LagPtr lfcn,
        // jacmlo, jacmup, astruc,
        0, 0, 0,
        // MatPtr afcn, RHSPtr ffcn, RHSPtr gfcn,
        NULL, CommonMHE::ffcn, NULL,
        // rwh,  iwh
        NULL, NULL
    );

    // define LSQ objective
    // NOTE: Measurements are defined for all nodes except the last one
    //       This is the crucial node, which is to be identified and is
    //       therefore left free to be optimized! The solution acts as new
    //       initial value for the NMPC MUSCOD instance.
    def_lsq(
        imos, "s", CommonMHE::NPR,
        CommonMHE::LSQFCN_FITTING_REG_NE, CommonMHE::lsqfcn_fitting_reg
    );
    def_lsq(
        imos, "i", CommonMHE::NPR,
        CommonMHE::LSQFCN_FITTING_REG_NE, CommonMHE::lsqfcn_fitting_reg
    );

    // increment model stage
    imos++;

    // check number of model stages
    CommonMHE::check_dimensions(0, 0, imos, CommonMHE::NMOS, __func__);
    // *********************************

    // define input and output methods
    // def_mio (NULL , NULL, NULL);
    // TODO: Add routine that creates RBDL models according to shooting nodes
    def_mio (
        CommonMHE::data_in , CommonMHE::meshup_output, CommonMHE::data_out
    );
}

// *****************************************************************************

