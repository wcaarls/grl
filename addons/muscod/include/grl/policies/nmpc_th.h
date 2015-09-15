/*
 * nmpc_th.h
 *
 *  Created on: Sep 12, 2015
 *      Author: ivan
 */

#ifndef NMPCPOLICYTH_H_
#define NMPCPOLICYTH_H_

#include <grl/policy.h>
#include "wrapper.hpp" // MUSCOD-II interface
#include <time.h>

namespace grl {

// Thread-safe data structure
struct MuscodData {
  unsigned long NMSN, NXD, NXA, NU, NP;
  bool is_initialized;
  bool quit;

  std::string model_path;
  std::string model_name;
  std::string relative_dat_path;

  std::vector<double> initial_sd; // only first shooting node contains initial values
  std::vector<double> initial_pf; // global parameters
  std::vector<std::vector<double> > qc; // all controls have to saved

  // TODO add threadsafe getter and setter for initial values and controls

  // Constructor
  MuscodData() :
    NMSN(0),
    NXD(0),
    NXA(0),
    NU(0),
    NP(0),
    quit(false),
    is_initialized(false)
  {}
};

class NMPCPolicyTh: public Policy
{
  public:
    TYPEINFO("policy/nmpc_th", "Nonlinear multistep (thread-based) model predictive control policy using the MUSCOD library")

  protected:
    std::string model_path_, model_name_;
    size_t outputs_, inputs_;
//    double step_;

    int verbose_;
 //   double  time_;
    int single_step_;

    // MUSCOD-II interface
    void *so_handle_; // hangle to a shared library with problem definitions
    void (*so_convert_obs_for_muscod)(const std::vector<double> &from, std::vector<double> &to);
    static MUSCOD *muscod_;
    Vector muscod_obs_;
    std::vector<Vector> muscod_action_;
    long   NP_, NMSN_, NXD_, NU_, NXA_;

    // pthread thread, conditions and mutexes
    void   print_array(const double* arr, const unsigned int len);
    void   muscod_init();
    void   muscod_reset();
    void   muscod_quit(void* data);
    static void*  muscod_run(void *indata);
    static bool   iv_ready_;
    static pthread_t muscod_thread_;
    static pthread_cond_t cond_iv_ready_;
    static pthread_mutex_t mutex_;
    MuscodData data_;
    unsigned int qc_cnt_, qc_cnt_base_;
    bool qc_ready_first_;

  public:
    NMPCPolicyTh(): single_step_(0), qc_cnt_(0), qc_cnt_base_(0), inputs_(0), outputs_(0) {};
    ~NMPCPolicyTh();

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual NMPCPolicyTh *clone() const;
    virtual void act(double time, const Vector &in, Vector *out);
};

} /* namespace grl */

#endif /* NMPCPOLICYTH_H_ */
