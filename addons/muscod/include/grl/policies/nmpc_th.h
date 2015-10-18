/*
 * nmpc_th.h
 *
 *  Created on: Sep 12, 2015
 *      Author: ivan
 */

#ifndef NMPCPOLICYTH_H_
#define NMPCPOLICYTH_H_

#include <grl/policy.h>
#include <grl/policies/common_code.h> // MUSCOD-II thread-safe data structure
#include "wrapper.hpp" // MUSCOD-II interface
#include <time.h>

namespace grl {

class NMPCPolicyTh: public Policy
{
  public:
    TYPEINFO("policy/nmpc_th", "Nonlinear multistep (thread-based) model predictive control policy using the MUSCOD library")

  protected:
    int verbose_, single_step_;
    std::string model_name_, lua_model_;
    size_t outputs_, inputs_;

    // MUSCOD-II interface
    void *so_handle_; // hangle to a shared library with problem definitions
    void (*so_convert_obs_for_muscod)(const double *from, double *to);
    static MUSCOD *muscod_;
    Vector muscod_obs_;
    Matrix muscod_action_;

    // pthread thread, conditions and mutexes
    void   print_array(const double* arr, const unsigned int len);
    void   muscod_reset(Vector &initial_obs, double time);
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
    NMPCPolicyTh(): single_step_(0), qc_cnt_(0), qc_cnt_base_(0), inputs_(0), outputs_(0), verbose_(0) {};
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
