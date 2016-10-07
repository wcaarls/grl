// *****************************************************************************
// Includes
// *****************************************************************************

#include <iostream>
#include <dlfcn.h>
#include <sys/stat.h>
#include <iomanip>

// GRL
#include <grl/policies/nmpc_stages.h>

// MUSCOD-II interface
#include <wrapper.hpp>

using namespace grl;

REGISTER_CONFIGURABLE(NMPCStagesPolicy);

TransitionType NMPCStagesPolicy::act(double time, const Vector &in, Vector *out)
{
  grl_assert((in.size() == nmpc_->NXD() + nmpc_->NP()) || (in.size() == nmpc_->NXD()));

  // subdivide 'in' into state and setpoint
  if (in.size() == nmpc_->NXD() + nmpc_->NP())
  {
    initial_sd_ << in.block(0, 0, 1, nmpc_->NXD());
    initial_pf_ << in.block(0, nmpc_->NXD(), 1, nmpc_->NP());
  } else {
    initial_sd_ << in;
  }

  if (verbose_)
  {
    std::cout << "time: [ " << time << " ]; state: [ " << initial_sd_ << "]" << std::endl;
    std::cout << "                          param: [ " << initial_pf_ << "]" << std::endl;
  }

  if (time == 0.0)
    muscod_reset(initial_sd_, initial_pf_, initial_qc_);

  out->resize( nmpc_->NU() );

  if (feedback_ == "non-threaded")
  {
    for (int inmpc = 0; inmpc < n_iter_; ++inmpc) {
      //std::cout << "NON-THREADED VERSION!" << std::endl;
      // 1) Feedback: Embed parameters and initial value from MHE
      // NOTE the same initial values (sd, pf) are embedded several time,
      //      but this will result in the same solution as running a MUSCOD
      //      instance for several iterations
      nmpc_->feedback(initial_sd_, initial_pf_, &initial_qc_);
      // 2) Shifting
      // NOTE do that only once at last iteration
      // NOTE this has to be done before the transition phase
      if (n_iter_ > 0 && inmpc == n_iter_-1) {
        nmpc_->shifting(1);
      }
      // 3) Transition
      nmpc_->transition();
      // 4) Preparation
      nmpc_->preparation();
    }
    // } // END FOR NMPC ITERATIONS
  }

  if (feedback_ == "threaded")
  {
    for (int inmpc = 0; inmpc < n_iter_; ++inmpc) {
        // std::cout << "THREADED VERSION!" << std::endl;
        // 1) Feedback: Embed parameters and initial value from SIMULATION
        // establish IPC communication to NMPC thread


        // NOTE do that only once at last iteration
        // NOTE this has to be done before the transition phase
        if (n_iter_ > 0 && inmpc == n_iter_ - 1) {
          nmpc_->set_shift_mode (1);
        } else {
          nmpc_->set_shift_mode (-1);
        }

        // provide iv to thread, get time until thread was ready
        iv_provided_ = true;
        provide_iv (nmpc_, initial_sd_, initial_pf_, &iv_provided_, false, verbose_);

        // provide iv to thread, get time until thread was ready
        if (iv_provided_)
          retrieve_qc (nmpc_, &initial_qc_, &qc_retrieved_, true, verbose_);

        // wait for preparation phase
        if (false) { // TODO Add wait flag
          wait_for_iv_ready(nmpc_, verbose_);
          if (nmpc_->get_iv_ready() == true) {
          } else {
              std::cerr << "MAIN: bailing out ..." << std::endl;
              abort();
          }
        }
    } // END FOR NMPC ITERATIONS
  }

  // Here we can return the feedback control
  // NOTE feedback control is cut of at action limits 'action_min/max'
  for (int i = 0; i < action_min_.size(); i++)
  {
    (*out)[i] = fmax( fmin(initial_qc_[i], action_max_[i]) , action_min_[i]);
    if ((*out)[i] != initial_qc_[i])
      WARNING("NMPC action " << i << " was truncated");
  }

  if (verbose_)
    std::cout << "Feedback Control: [" << *out << "]" << std::endl;

  return ttGreedy;
}

