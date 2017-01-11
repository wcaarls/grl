/*
 * nmpc_th.h
 *
 *  Created on: Sep 08, 2016
 *      Author: mkudruss
 */

#ifndef NMPCPOLICY_MLRTI_H_
#define NMPCPOLICY_MLRTI_H_

#include <grl/policy.h>
#include <grl/policies/muscod_data.h> // MUSCOD-II thread-safe data structure
#include <grl/policies/nmpc_base.h>
#include "wrapper.hpp" // MUSCOD-II interface
#include "muscod_nmpc.h" // MUSCOD-II NMPCProblem class
#include <time.h>

namespace grl {

class NMPCPolicyMLRTI: public NMPCBase
{
  public:
    TYPEINFO(
        "policy/nmpc_mlrti",
        "Thread-based nonlinear model predictive control policy using the MUSCOD library and implementing a multi-level real-time iteration "
    )

  protected:
    int nmpc_ninit_; // number of MUSCOD SQP iterations for initialization
    Vector initial_pf_, initial_qc_, final_sd_;

    MUSCOD *muscod_A_;
    NMPCProblem *nmpc_A_;
    const std::string thread_id_A = "A";
    pthread_t thread_A_;
    pthread_cond_t cond_iv_ready_A_;
    pthread_mutex_t mutex_A_;

    MUSCOD *muscod_B_;
    NMPCProblem *nmpc_B_;
    const std::string thread_id_B = "B";
    pthread_t thread_B_;
    pthread_cond_t cond_iv_ready_B_;
    pthread_mutex_t mutex_B_;

    // pointer to different NMPC modes
    NMPCProblem* cntl_;
    NMPCProblem* idle_;

    Vector muscod_obs_;
    Matrix muscod_action_;

    // relative path to model directory
    // NOTE will end up next to the DAT file as 'run_nmpc.bin'!
    const std::string restart_path_ = "";
    const std::string restart_name_ = "run_nmpc";

    // state machine to handle multi-level real-time iteration:
    // 0: idle_call
    //    call idle at current state, retrieve feedback, start re-linearization
    //    state -> 1 (idle_ready)
    // 1: idle_ready:
    //    while waiting for the preparation phase of idle, provide feedback
    //    from cntl, if idle is ready state -> 2 (idle_switch)
    // 2: idle_switch
    //    switch controllers idle <-> cntl
    //    state -> 0 (idle_call)
    enum STATE {
        idle_call = 0,
        idle_ready,
        idle_switch,
        STATE_LAST
    };
    STATE current_state_;

    // temporary variable to control IPC to controller threads
    // NOTE iv_provided and qc_retrieved booleans are options for get_feedback
    //      convenience function
    // NOTE *_iv_provided is INPUT and OUTPUT, if false on INPUT then skip
    //      iv part (provide IV and initialize Feedback computation)
    // NOTE *_qc_retrieved is INPUT and OUTPUT, if false on INPUT then skip
    //      qc part (Feedback and get QC from thread)
    bool idle_iv_provided_;
    bool idle_qc_retrieved_;
    bool cntl_iv_provided_;
    bool cntl_qc_retrieved_;

    // joint timing statistics
    // NOTE ttimer copies respective data from controller timings
    MUSCODTiming ttimer_;

    // define variables to time NMPC execution
    std::vector<Vector> timing_values_;
    std::vector<Vector> timing_values_idle_;
    std::vector<Vector> timing_values_cntl_;

    // pthread thread, conditions and mutexes
    void   print_array(const double* arr, const unsigned int len);
    void   muscod_reset(const Vector &initial_obs, double time);
    void   muscod_quit(void* data);

  public:
    NMPCPolicyMLRTI():
        nmpc_ninit_(10),
        idle_iv_provided_ (true),
        idle_qc_retrieved_ (true),
        cntl_iv_provided_ (true),
        cntl_qc_retrieved_ (true),
        nmpc_A_(NULL),
        nmpc_B_(NULL),
        muscod_A_(NULL),
        muscod_B_(NULL)
    {}

    ~NMPCPolicyMLRTI();

    // From Configurable
    virtual void request(ConfigurationRequest *config);
    virtual void configure(Configuration &config);
    virtual void reconfigure(const Configuration &config);

    // From Policy
    virtual TransitionType act(double time, const Vector &in, Vector *out);
};

} /* namespace grl */

#endif /* NMPCPOLICY_MLRTI_H_ */
