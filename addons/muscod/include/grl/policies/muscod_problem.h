// *****************************************************************************
// MUSCOD-II Problem Data Implementation
//
// A wrapper for MUSCOD problem data
//
// Copyright (C) 2016, Manuel Kudruss, IWR, Heidelberg
// *****************************************************************************

// *****************************************************************************
// Includes
// *****************************************************************************

// GRL
#include <grl/utils.h>

// C++ standard library
#include <dlfcn.h> // to handle dynamic libraries
#include <iostream> // for standard cout, cerr

// threading support
#include <pthread.h>

// Eigen3 support
#include <Eigen/Dense>

// MUSCOD-II interface
#include "wrapper.hpp"

// *****************************************************************************
#ifndef MUSCOD_PROBLEM_H_
#define MUSCOD_PROBLEM_H_
// *****************************************************************************
// namespace MUSCODProblem { // BEGIN NAMESPACE MUSCOD
// *****************************************************************************

struct MUSCODTiming {

    // vector of timing data
    // of single operations
    // 0: feedback
    // 1: transition
    // 2: shift
    // 3: prepare

    // of combined operations
    // 4: preparation
    // 5: total
    grl::Vector _timing;

    void time_feedback (double timing) {_timing[0] = timing;}
    void time_transition (double timing) {_timing[1] = timing;}
    void time_shift (double timing) {_timing[2] = timing;}
    void time_prepare (double timing) {_timing[3] = timing;}
    void time_preparation (double timing) {_timing[4] = timing;}
    void time_total (double timing) {_timing[5] = timing;}

    double get_feedback () {return _timing[0];}
    double get_transition () {return _timing[1];}
    double get_shift () {return _timing[2];}
    double get_prepare () {return _timing[3];}
    double get_preparation () {return _timing[4];}
    double get_total () {return _timing[5];}

    MUSCODTiming ()
    {
        // initialize vectors
        _timing = grl::Vector::Zero(6);
    };

    void reset() {
      _timing.fill(0.0);
    }

};


// *****************************************************************************
// MUSCOD data base class
// *****************************************************************************

struct MUSCODProblem {
  bool m_verbose;

  // thread handles
  std::string thread_id;
  pthread_cond_t* cond_iv_ready_;
  pthread_mutex_t* mutex_;

  // MUSCOD-II handles
  bool m_extern_muscod;
  std::string m_problem_path, m_model_name;
  MUSCOD* m_muscod; // pointer to MUSCOD process
  CHMData* m_data; // pointer to MUSCOD data
  TOptions* m_options; // pointer to MUSCOD options

  // MUSCOD-II timing statistics of NMPC evaluations
  MUSCODTiming timing;

  // problem library handles
  void* m_problem_so_handle; // handle to problem library

  // symbols from problem library
  bool (*so_get_plot_counter) ();
  void (*so_set_plot_counter) (bool counter);

  // problem dimensions
  unsigned long NMOS_, NMSN_, NXD_, NXA_, NU_, NP_, NH_;
  grl::Vector nmsn_;

  unsigned long NMOS() {return NMOS_;}
  unsigned long NMSN() {return NMSN_;}
  unsigned long NMSN(long imos) {return nmsn_[imos];}
  unsigned long NXD() {return NXD_;}
  unsigned long NXA() {return NXA_;}
  unsigned long NU() {return NU_;}
  unsigned long NP() {return NP_;}
  unsigned long NH() {return NH_;}

  // ---------------------------------------------------------------------------

  /*
  void print_states () {
    // grl::Vector times (m_NMSN);
    // grl::MatrixXd states (m_NXD, m_NMSN);
    // set up measurements on horizon
    for (int imsn = 0; imsn < m_NMSN; ++imsn) {
      m_muscod->getNodeSD(imsn, states.col(imsn).data());
    }
    // for (int imsn = 0; imsn < m_NMSN; ++imsn) {
    //   m_muscod->getNodeSD(imsn, states.col(imsn).data());
    // }
    std::cout << "states = " << std::endl << states << std::endl;
  }

  void print_controls () {
    // grl::MatrixXd controls (m_NU, m_NMSN);
    // set up measurements on horizon
    for (int imsn = 0; imsn < m_NMSN; ++imsn) {
      m_muscod->getNodeQC(imsn, controls.col(imsn).data());
    }
    std::cout << "controls = " << std::endl << controls << std::endl;
  }

  void print_parameters () {
    grl::Vector parameters (m_NP);
    m_muscod->getPF(parameters.data());
    std::cout << "parameters = " << std::endl << parameters << std::endl;
  }
  */

  // ---------------------------------------------------------------------------

  void feedback(
    const grl::Vector& initial_sd,
    const grl::Vector& initial_pf,
    grl::Vector* first_qc
  ) {
    m_muscod->nmpcFeedback(
      initial_sd.data(), initial_pf.data(), first_qc->data()
    );
  }

  void feedback(
    const grl::Vector& initial_sd,
    grl::Vector* first_qc
  ) {
    m_muscod->nmpcFeedback(initial_sd.data(), NULL, first_qc->data());
  }

  void feedback() {
    m_muscod->nmpcFeedback(NULL, NULL, NULL);
  }

  void transition() {
    m_muscod->nmpcTransition();
  }

  void preparation() {
    m_muscod->nmpcPrepare();
  }

  void shifting(long shift_mode) {
    m_muscod->nmpcShift(shift_mode);
  }

  void simulate(
    const grl::Vector& initial_sd,
    const grl::Vector& initial_pf,
    const grl::Vector& initial_qc,
    const double& integration_time,
    grl::Vector* final_sd
  ) {
    m_muscod->nmpcSimulate(
      1,               // simulate, no derivatives
      integration_time,           // time interval
      initial_sd.data(),  // take current initial value
      NULL,            // no algebraic states
      initial_qc.data(),    // use feedback control
      initial_pf.data(),  // no parameters involved
      final_sd->data(),
      NULL,   // final_sa
      NULL,   // final_sd_sd0
      NULL,   // final_sd_q
      NULL,   // final_sd_pf
      NULL,   // final_sa_sd0
      NULL,   // final_sa_q
      NULL    // final_sa_pf
    );
  }

  double getSamplingRate () {
    double ptime;
    m_muscod->getNodePhystime(0, ptime, false);
    return ptime;
  }

  // ---------------------------------------------------------------------------

  ~MUSCODProblem () {
    // NOTE: Use that order, because data and options are members of m_muscod
    if (m_extern_muscod) {
      m_muscod = NULL;
    } else {
      delete m_muscod;
    }

    // close shared library
    so_get_plot_counter = NULL;
    so_set_plot_counter = NULL;

    std::string so_path  = m_problem_path + "/" + "lib" + m_model_name + ".so";
    if (m_verbose) {
      std::cout << "unloading shared library";
      std::cout << so_path << "'" << std::endl;
    }
    dlclose(m_problem_so_handle);
  }

  // ---------------------------------------------------------------------------

  MUSCODProblem(
    std::string problem_path, std::string model_name,
    MUSCOD* muscod = NULL
  ) :
    m_verbose (false),
    cond_iv_ready_ (NULL),
    mutex_ (NULL),
    m_extern_muscod(false),
    thread_id ("")
  {
    // assign model and problem path
    m_problem_path = problem_path;
    m_model_name = model_name;

    // reference or create new MUSCOD-II instance
    if (!muscod) {
      m_extern_muscod = false;
      m_muscod = new MUSCOD;
    } else {
      m_extern_muscod = true;
      m_muscod = muscod;
    }

    // assign data and options
    m_data = &(m_muscod->data); // is reference
    m_options = m_muscod->options; // is already a pointer

    m_muscod->setModelPathAndName(m_problem_path.c_str(), m_model_name.c_str());
    m_muscod->loadFromDatFile(NULL, NULL);
    m_muscod->nmpcInitialize(0, NULL, NULL);

    // get dimensions from MUSCOD instance
    get_NMOS();
    get_NMSN();
    get_NXD_NXA_NU();
    get_NP();
    get_NH();

    // load shared symbols from problem library
    std::string so_path  = m_problem_path + "/" + "lib" + m_model_name + ".so";
    m_problem_so_handle = load_so(so_path);

    so_get_plot_counter = (bool (*) ())
      load_symbol(m_problem_so_handle, "get_plot_counter");

    so_set_plot_counter = (void (*) (bool))
      load_symbol(m_problem_so_handle, "set_plot_counter");

  }

  // ---------------------------------------------------------------------------

  void* load_so(std::string so_path){
    // load shared library
    void *so_handle;
    so_handle = dlopen(so_path.c_str(), RTLD_NOW|RTLD_GLOBAL);
    if (so_handle == NULL)
    {
      std::cout << "ERROR: Could not load shared library: '";
      std::cout << so_path << "'" << std::endl;
      std::cout << "dlerror response: " << dlerror() << std::endl;
      std::cout << "bailing out ..." << std::endl;
      exit(EXIT_FAILURE);
    }

    std::cout << "Successfully loaded shared library: '";
    std::cout << so_path << "'" << std::endl;
    return so_handle;
  }

  void* load_symbol(void *so_handle, std::string symbol_name){
    // load shared library
    if (so_handle == NULL) {
      std::cout << "ERROR: No library handle provided!" << std::endl;
      exit(EXIT_FAILURE);
    }

    void* symbol;
    dlerror(); // reset last error message
    symbol = dlsym(so_handle, symbol_name.c_str());

    std::string err_msg;
    char* error;
    if ((error = dlerror()) != NULL)  {
      std::cout << "ERROR: Could not load symbol: '";
      std::cout << symbol_name << "'" << std::endl;
      std::cout << "dlerror response: " << err_msg << std::endl;
      std::cout << "bailing out ..." << std::endl;
      exit(EXIT_FAILURE);
    }

    return symbol;
  }

  bool get_plot_counter(){
    return (bool) so_get_plot_counter();
  }

  void set_plot_counter(bool choice){
    so_set_plot_counter(choice);
  }

  // ---------------------------------------------------------------------------

  void get_NMOS () {
    // get Problem dimensions
    NMOS_ = m_muscod->getNMOS();
    // if (NMOS_ != 1) {
    //   std::cerr << "In " << __func__ << ": ";
    //   // Begin ERROR MESSAGE
    //   std::cerr << "more then one stage is not allowed for NMPC";
    //   std::cerr << std::endl;
    //   // End ERROR MESSAGE
    //   std::cerr << "bailing out..." << std::endl;
    //   exit(EXIT_FAILURE);
    // };
  }

  void get_NMSN () {
    // get number of shooting intervals
    nmsn_ = grl::Vector::Zero(NMOS_);
    for (int imos = 0; imos < NMOS_; ++imos) {
      nmsn_[imos] = m_muscod->getNMSN(imos);
    }
    NMSN_ = nmsn_.sum();
    if (NMSN_ <= 0) {
      std::cerr << "In " << __func__ << ": ";
      // Begin ERROR MESSAGE
      std::cerr << "could not get number of shooting nodes from MUSCOD";
      std::cerr << std::endl;
      // End ERROR MESSAGE
      std::cerr << "bailing out..." << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  void get_NXD_NXA_NU (long imos=0) {
    // get state and control dimensions
    if (m_muscod->getDimIMSN(imos, &NXD_, &NXA_, &NU_) != 0) {
      std::cerr << "In " << __func__ << ": ";
      // Begin ERROR MESSAGE
      std::cerr << "could not get problem dimensions from MUSCOD";
      std::cerr << std::endl;
      // End ERROR MESSAGE
      std::cerr << "bailing out..." << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  void get_NP () {
    // get number of parameters and horizon length
    if ((m_muscod->getNP(&NP_) < 0)) {
      std::cerr << "In " << __func__ << ": ";
      // Begin ERROR MESSAGE
      std::cerr << "could not get NP from MUSCOD";
      std::cerr << std::endl;
      // End ERROR MESSAGE
      std::cerr << "bailing out..." << std::endl;
      abort();
    }
  }

  void get_NH () {
    // get number of parameters and horizon length
    if ((m_muscod->getNH(&NH_) < 0)) {
      std::cerr << "In " << __func__ << ": ";
      // Begin ERROR MESSAGE
      std::cerr << "could not get NH from MUSCOD";
      std::cerr << std::endl;
      // End ERROR MESSAGE
      std::cerr << "bailing out..." << std::endl;
      abort();
    }
  }

    void getHF (grl::Vector* hf) {
        // get number of parameters and horizon length
        m_muscod->getHF((*hf).data());
    }

    void setHF (
        grl::Vector& hf
    ){
        m_muscod->setHF(hf.data());
    }

    void getNodeSD (
        const unsigned long imsn,
        grl::Vector* sd
    ){
        m_muscod->getNodeSD(imsn, (*sd).data());
    }

    void setNodeSD (
        const unsigned long imsn,
        grl::Vector& sd
    ){
        m_muscod->setNodeSD(imsn, sd.data());
    }

    void getNodeQC (
        const unsigned long imsn,
        grl::Vector* qc
    ){
        m_muscod->getNodeQC(imsn, (*qc).data());
    }

    void setNodeQC (
        const unsigned long imsn,
        grl::Vector& qc
    ){
        m_muscod->setNodeQC(imsn, qc.data());
    }

}; // END MUSCODProblem

// *****************************************************************************
// } // END NAMESPACE MUSCOD
// *****************************************************************************
// #endif MUSCOD_PROBLEM_H_
#endif
// *****************************************************************************

