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
std::vector<RBDLModel> rbdl_models;

// *****************************************************************************
// Constants
// *****************************************************************************

std::string model_file_name = (std::string) "./model.lua";
const std::string ref_states_path = (std::string) "RES/sd_cart_pendulum_time.csv";
const std::string ref_cntrls_path = (std::string) "RES/u_cart_pendulum_time.csv";

// *****************************************************************************
// Variables
// *****************************************************************************

bool is_problem_name_initialized = false;
bool is_tikhonov_initialized = false;
bool is_horizon_initialized = false;
bool add_counter = false;
char problem_name[255];
long real_nmos, real_nmsn;
double real_tmax;

std::string rel_data_path = "";

// Tikhonov regularization
Eigen::VectorXd p0(NP);
Eigen::VectorXd s0(NP);

// placeholder for plotting
std::vector<double> _plotting_t_values;
std::vector<double> _plotting_p_values;
std::vector<std::vector<double> > _plotting_sd_values;
std::vector<std::vector<double> > _plotting_u_values;

std::vector<double> _plotting_ts_values;
std::vector<std::vector<double> > _plotting_hs_values;
std::vector<std::vector<double> > _plotting_ss_values;

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
// Least Squares Functions
// *****************************************************************************

const unsigned int LSQFCN_FITTING_NE = NXD + NU;
void lsqfcn_fitting(
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
    bool  p_dpnd = false;
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
    // std::cout << "cnode = " << info->cnode << std::endl;
    // std::cout << "real_nmsn = " << real_nmsn << std::endl;
    meas_hs = Eigen::MatrixXd::Zero(NXD + NU, real_nmsn+1);
    meas_ss = Eigen::MatrixXd::Ones(NXD + NU, real_nmsn+1);
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

  // check dimensions of function
  check_dimensions(0, 0, res_ne_cnt, LSQFCN_FITTING_NE, __func__);
}

const unsigned int MSQFCN_FITTING_NE = NXD;
void msqfcn_fitting(
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
    bool  u_dpnd = false;
    bool  p_dpnd = false;
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
    // std::cout << "cnode = " << info->cnode << std::endl;
    // std::cout << "real_nmsn = " << real_nmsn << std::endl;
    meas_hs = Eigen::MatrixXd::Zero(NXD + NU, real_nmsn+1);
    meas_ss = Eigen::MatrixXd::Ones(NXD + NU, real_nmsn+1);
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

  // check dimensions of function
  check_dimensions(0, 0, res_ne_cnt, MSQFCN_FITTING_NE, __func__);
}

void initialize_tikhonov (){
    meas_hs = Eigen::MatrixXd::Zero(NXD + NU, real_nmsn+1);
    meas_ss = Eigen::MatrixXd::Ones(NXD + NU, real_nmsn+1);
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
    // std::cout << "cnode = " << info->cnode << std::endl;
    // std::cout << "real_nmsn = " << real_nmsn << std::endl;
    initialize_tikhonov ();
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
    s0 << 10.0;
  }
  res[res_ne_cnt++] = (p[0] - p0[0]) / s0[0];

  // check dimensions of function
  check_dimensions(0, 0, res_ne_cnt, LSQFCN_FITTING_REG_NE, __func__);
}

const unsigned int MSQFCN_FITTING_REG_NE = 4 + 1;
void msqfcn_fitting_reg(
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
    bool  u_dpnd = false;
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
    // std::cout << "cnode = " << info->cnode << std::endl;
    // std::cout << "real_nmsn = " << real_nmsn << std::endl;
    initialize_tikhonov ();
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

  // Tikhonov regularization
  // || (p - p0) / s0 ||
  if (!is_tikhonov_initialized) {
    p0 << 0.0;
    s0 << 1000.0;
  }
  res[res_ne_cnt++] = (p[0] - p0[0]) / s0[0];

  // check dimensions of function
  check_dimensions(0, 0, res_ne_cnt, MSQFCN_FITTING_REG_NE, __func__);
}

// *****************************************************************************
// Right Hand Sides
// *****************************************************************************

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
  real_tmax = std::max(real_tmax, *h);

  // std::cout << "real_nmos = " << real_nmos << std::endl;
  // std::cout << "real_nmsn = " << real_nmsn << std::endl;
  // std::cout << "real_tmax = " << real_tmax << std::endl;

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
      for (unsigned int i = 0; i < _plotting_ts_values.size(); i ++) {
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

      if (add_counter) {
        iteration_cnt++;
      }
      lnode = -1;
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
  }

  if (info->cnode != lnode) {
      // std::cout << "cnode = " << info->cnode << std::endl;
      // std::cout << "t     = " << *t << std::endl;
      // std::cout << "cols  = " << meas_hs.cols() << std::endl;

    if (info->cnode == 0) {
      for (unsigned i = 0; i < NP; i++) {
        _plotting_p_values.push_back(p[i]);
      }
    }

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
    // set lnode to current node
    lnode = info->cnode;
  }
  if (*t == real_tmax) {
      lnode = real_nmsn;
      // std::cout << "cnode = " << lnode << std::endl;
      // std::cout << "t     = " << *t << std::endl;
      // std::cout << "cols  = " << meas_hs.cols() << std::endl;

      _plotting_ts_values.push_back (*t);

      std::vector<double> _hs_vec (NXD + NU);
      for (unsigned i = 0; i < NXD + NU; i++){
        _hs_vec[i] = meas_hs(i, lnode);
      }
      _plotting_hs_values.push_back (_hs_vec);

      std::vector<double> _ss_vec (NXD + NU);
      for (unsigned i = 0; i < NXD + NU; i++){
        _ss_vec[i] = meas_ss(i, lnode);
      }
      _plotting_ss_values.push_back (_ss_vec);
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
    const bool use_tikhonov_regularization = false;
    if (use_tikhonov_regularization) {
        def_lsq(
            imos, "s", CommonMHE::NPR,
            CommonMHE::LSQFCN_FITTING_REG_NE, CommonMHE::lsqfcn_fitting_reg
        );
        def_lsq(
            imos, "i", CommonMHE::NPR,
            CommonMHE::LSQFCN_FITTING_REG_NE, CommonMHE::lsqfcn_fitting_reg
        );
        def_lsq(
            imos, "e", CommonMHE::NPR,
            CommonMHE::MSQFCN_FITTING_REG_NE, CommonMHE::msqfcn_fitting_reg
        );
    } else {
        def_lsq(
            imos, "s", CommonMHE::NPR,
            CommonMHE::LSQFCN_FITTING_NE, CommonMHE::lsqfcn_fitting
        );
        def_lsq(
            imos, "i", CommonMHE::NPR,
            CommonMHE::LSQFCN_FITTING_NE, CommonMHE::lsqfcn_fitting
        );
        def_lsq(
            imos, "e", CommonMHE::NPR,
            CommonMHE::MSQFCN_FITTING_NE, CommonMHE::msqfcn_fitting
        );
    }

    // increment model stage
    imos++;

    // check number of model stages
    CommonMHE::check_dimensions(0, 0, imos, CommonMHE::NMOS, __func__);
    // *********************************

    // define input and output methods
    def_mio (
        CommonMHE::data_in , NULL, NULL //CommonMHE::meshup_output, CommonMHE::data_out
    );
}

// *****************************************************************************

