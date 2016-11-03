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
  // NOTE this time it's different NXD is larger than the values in 'in'
  std::cout << "in.size() = "    << in.size() << std::endl;
  std::cout << "nmpc_->NXD() = " << nmpc_->NXD() << std::endl;
  grl_assert(
    (in.size() == nmpc_->NXD()+1) || (in.size() == nmpc_->NXD())
    || (in.size() == nmpc_->NXD()+2)
  );

  // subdivide 'in' into state and set-point
  // TODO subdivide into actual states and switch statement
  // if (in.size() == nmpc_->NXD() + nmpc_->NP())
  // {
    std::cout << "in:  [ " << in << "]" << std::endl;
    // initial_sd_.block(0, 0, 1, nmpc_->NXD()-1) << in.block(0, 0, 1, nmpc_->NXD()-1);

    initial_sd_[0] = in[0];
    initial_sd_[1] = in[1] + 2* in[0];
    initial_sd_[2] = in[2];
    initial_sd_[3] = in[3] + 2* in[2];

    if (in.size() == nmpc_->NXD())
    {
      std::cout << "AVG VELOCITY?" << std::endl;
      initial_sd_[4] = in[5]; // v_h
    }

    if (in.size() == nmpc_->NXD()+2)
    {
      // last entry is always time
      initial_sd_[nmpc_->NXD()-1] = 0.0; // time is always zero for NMPC!
    }

    // FIXME
    initial_hf_.resize(nmpc_->NH());

    initial_swt_.resize(1);
    initial_swt_ << in.block(0, nmpc_->NXD()-1, 1, 1 /*one switch*/);
  // } else {
  //   initial_sd_ << in;
  // }

  // if (verbose_)
  // {
    std::cout << "time: [ " << time << " ]" << std::endl;
    std::cout << "state:  [ " << initial_sd_ << "]" << std::endl;
    std::cout << "switch: [ " << initial_swt_ << "]" << std::endl;
  // }

  // if (time == 0.0)
  //   muscod_reset(initial_sd_, initial_qc_);

  out->resize( nmpc_->NU() );


  if (true) {
    initial_qc_ << Vector::Zero (nmpc_->NU());
    nmpc_->simulate(initial_sd_, initial_qc_, 0.2, &final_sd_);
    std::cout << "final:  [ " << final_sd_ << "]" << std::endl;
  } else  {

  // retrieve current stage durations
  nmpc_->getHF(&initial_hf_);

  // if (verbose) {
  std::cout << "initial_hf: [ " << initial_hf_ << " ]" << std::endl;
  // }

  //  FIXME get control step here!
  if ( initial_hf_[0] < 0.2 ) {
    std::cout << "BOOM!" << std::endl;
    if ( initial_swt_[0] == 1 ) {
      std::cout << "SWITCH DETECTED!" << std::endl;
      roll_out = true;
      run_nmpc = true;
    } else {
      std::cout << "*NO* SWITCH DETECTED!" << std::endl;
      roll_out = false;
      run_nmpc = true;
    }
  }

  if (roll_out)
  {
    // set transition duration to limit cycle duration
    // NOTE first feedback phase has fixed duration!
    initial_hf_[0] = initial_hf_[2]; // reinitialize feedback control stage
    initial_hf_[1] = initial_hf_[3]; // update transition with limit cycle length
    nmpc_->setHF(initial_hf_);

    // dummy quantities for boot strapping
    Vector sd_limit = Vector(nmpc_->NXD());
    Vector sd_trans = Vector(nmpc_->NXD());
    Vector qc_dummy = Vector(nmpc_->NXD());

    // shooting node offsets
    int offset_trans;
    int offset_limit;
    // Roll out limit cycle to
    // 0) feedback stage (only a singe shooting node)
    offset_trans = nmpc_->nmsn_.head(0).sum();
    offset_limit = nmpc_->nmsn_.head(3).sum();
    for (int imsn = 0; imsn < nmpc_->NMSN(0); ++imsn) {
      // STATES
      // get respective states from limit cycle
      nmpc_->getNodeSD(imsn + offset_limit, &sd_limit);

      // get respective states from limit cycle
      nmpc_->getNodeSD(imsn + offset_trans, &sd_trans);

      // copy over data
      // NOTE check for application if all states have to be
      //      copied over, for a bouncing ball only z component
      //      lies in periodic orbit
      sd_trans << sd_limit;

      // set respective states in transition phase
      nmpc_->setNodeSD(imsn + offset_trans, sd_trans);

      // CONTROLS
      // get respective states from limit cycle
      nmpc_->getNodeQC(imsn + offset_limit, &qc_dummy);
      // set respective states in transition phase
      nmpc_->setNodeQC(imsn + offset_trans, qc_dummy);
    }

    // 1) transition stage (several shooting nodes)
    offset_trans = nmpc_->nmsn_.head(1).sum();
    offset_limit = nmpc_->nmsn_.head(4).sum();
    for (int imsn = 0; imsn < nmpc_->NMSN(1); ++imsn) {
      // STATES
      // get respective states from limit cycle
      nmpc_->getNodeSD(imsn + offset_limit, &sd_limit);

      // get respective states from limit cycle
      nmpc_->getNodeSD(imsn + offset_trans, &sd_trans);

      // copy over data
      // NOTE check for application if all states have to be
      //      copied over, for a bouncing ball only z component
      //      lies in periodic orbit
      sd_trans << sd_limit;

      // set respective states in transition phase
      nmpc_->setNodeSD(imsn + offset_trans, sd_trans);

      // CONTROLS
      // get respective states from limit cycle
      nmpc_->getNodeQC(imsn + offset_limit, &qc_dummy);
      // set respective states in transition phase
      nmpc_->setNodeQC(imsn + offset_trans, qc_dummy);
    }

    // 2) switching phase
    offset_trans = nmpc_->nmsn_.head(2).sum();
    offset_limit = nmpc_->nmsn_.head(5).sum();
    for (int imsn = 0; imsn < nmpc_->NMSN(2); ++imsn) {
      // STATES
      // get respective states from limit cycle
      nmpc_->getNodeSD(imsn + offset_limit, &sd_limit);

      // get respective states from limit cycle
      nmpc_->getNodeSD(imsn + offset_trans, &sd_trans);

      // copy over data
      // NOTE check for application if all states have to be
      //      copied over, for a bouncing ball only z component
      //      lies in periodic orbit
      sd_trans << sd_limit;

      // set respective states in transition phase
      nmpc_->setNodeSD(imsn + offset_trans, sd_trans);

      // CONTROLS
      // get respective states from limit cycle
      nmpc_->getNodeQC(imsn + offset_limit, &qc_dummy);
      // set respective states in transition phase
      nmpc_->setNodeQC(imsn + offset_trans, qc_dummy);
    }

    // set respective states in transition phase
    nmpc_->setNodeSD(0, initial_sd_);
    muscod_nmpc_->nmpcEmbedding(initial_sd_.data(), NULL);

    // Preparation to build up new QP
    // NOTE update linearization point by copying var to varnew
    TDataMSSQP *d = MCData->dataMSSQP;
    copy_PVars(&d->var, &d->varnew);
    nmpc_->preparation();

    for (int i = 0; i < 1; ++i) {
      nmpc_->feedback(initial_sd_, &initial_qc_);
      nmpc_->transition();
      nmpc_->preparation();
    }
  }

  if (run_nmpc)
  {
    // if (feedback_ == "non-threaded")
    // {
      for (int inmpc = 0; inmpc < n_iter_; ++inmpc) {
        //std::cout << "NON-THREADED VERSION!" << std::endl;
        // 1) Feedback: Embed parameters and initial value from MHE
        // NOTE the same initial values (sd, pf) are embedded several time,
        //      but this will result in the same solution as running a MUSCOD
        //      instance for several iterations
        nmpc_->feedback(initial_sd_, &initial_qc_);
        // 2) Shifting
        // NOTE do that only once at last iteration
        // NOTE this has to be done before the transition phase
        // if (n_iter_ > 0 && inmpc == n_iter_-1) {
        //   nmpc_->shifting(-1);
        // }
        // 3) Transition
        nmpc_->transition();
        // 4) Preparation
        nmpc_->preparation();
      }
      // } // END FOR NMPC ITERATIONS
    // }

    // if (feedback_ == "threaded")
    // {
    /*
      // TODO NOT TESTED YET
      abort();
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
    */
    // }
  } else {
    std::cout << "NMPC skipped!" << std::endl;
  }

  // re-enable flag
  run_nmpc = true;
  // initial_qc_ << Vector::Zero (nmpc_->NU());
  }

  // Here we can return the feedback control
  // NOTE feedback control is cut of at action limits 'action_min/max'
  for (int i = 0; i < action_min_.size(); i++)
  {
    (*out)[i] = fmax( fmin(initial_qc_[i], action_max_[i]) , action_min_[i]);
    if ((*out)[i] != initial_qc_[i])
      WARNING("NMPC action " << i << " was truncated");
  }

  // if (verbose_)
    std::cout << "Feedback Control: [" << *out << "]" << std::endl;

  return ttGreedy;
}

