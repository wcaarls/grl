// *****************************************************************************
// Includes
// *****************************************************************************

#include <iostream>
#include <dlfcn.h>
#include <sys/stat.h>
#include <iomanip>

// GRL
#include <grl/policies/mhe_nmpc.h>

// MUSCOD-II interface
#include <wrapper.hpp>

using namespace grl;

REGISTER_CONFIGURABLE(MHE_NMPCPolicy)

MHE_NMPCPolicy::~MHE_NMPCPolicy()
{
  // stop threads
  if (nmpc_)
    stop_thread (*nmpc_, &thread_);

  // safely delete instances
  safe_delete(&nmpc_);
  safe_delete(&muscod_nmpc_);
  safe_delete(&mhe_);
  safe_delete(&muscod_mhe_);
}

void MHE_NMPCPolicy::request(ConfigurationRequest *config)
{
  NMPCBase::request(config);
  config->push_back(CRP("mhe_model_name", "Name of MUSCOD MHE model library", ""));
  config->push_back(CRP("feedback", "Choose between a non-treaded and a threaded feedback of NMPC", feedback_, CRP::Configuration, {"non-threaded", "threaded"}));
  config->push_back(CRP("n_iter", "Number of iteration", (int)n_iter_, CRP::System, 0, INT_MAX));
}

void MHE_NMPCPolicy::configure(Configuration &config)
{
  NMPCBase::configure(config);
  feedback_ = config["feedback"].str();
  n_iter_ = config["n_iter"];

  INFO("Running " << feedback_ << " implementation of NMPC");

  std::string mhe_model_name   = config["mhe_model_name"].str();

  // Setup path for the problem description library and lua, csv, dat files used by it
  std::string problem_path  = model_path_ + "/" + model_name_;

  //-------------------- Load Lua model which is used by muscod ------------------- //
  if (!config["lua_model"].str().empty())
  {
    lua_model_ = problem_path + "/" + config["lua_model"].str();

    struct stat buffer;
    if (stat(lua_model_.c_str(), &buffer) != 0) // check if lua file exists in the problem description folder
      lua_model_ = std::string(RBDL_LUA_CONFIG_DIR) + "/" + config["lua_model"].str(); // if not, then use it as a reference from dynamics
  }
  else
    lua_model_ = "";
  //----------------- Set path in the problem description library ----------------- //
  setup_model_path(problem_path, mhe_model_name, lua_model_);
  setup_model_path(problem_path, nmpc_model_name_, lua_model_);

  //------------------- Initialize MHE ------------------- //
  muscod_mhe_ = new MUSCOD();
  mhe_ = new MHEProblem(problem_path.c_str(), mhe_model_name.c_str(), muscod_mhe_);

  Vector hs(mhe_->NXD() + mhe_->NU()), ss(mhe_->NXD() + mhe_->NU()), qs(mhe_->NU());
  hs << 0.00, 3.14159, 0.00, 0.00, 0.00; // hanging down
  ss << 1.00, 1.00,    1.00, 1.00, 0.10; // no error
  qs << 0.00; // no control

  // initialize measurement horizon with data
  mhe_->initialize_horizon(hs, ss, qs);

  //------------------- Initialize NMPC ------------------- //
  muscod_nmpc_ = new MUSCOD();
  if (verbose_) {
    muscod_nmpc_->setLogLevelAndFile(-1, NULL, NULL);
  } else {
    muscod_nmpc_->setLogLevelTotal(-1);
  }
  nmpc_ = new NMPCProblem(problem_path.c_str(), nmpc_model_name_.c_str(), muscod_nmpc_);
  if (verbose_) {
    nmpc_->m_verbose = true;
  } else {
    nmpc_->m_verbose = false;
  }

  // provide condition variable and mutex to NMPC instance
  nmpc_->cond_iv_ready_ = &cond_iv_ready_;
  nmpc_->mutex_ = &mutex_;

  // start NMPC controller in own thread running signal controlled event loop
  initialize_thread(
    &thread_, muscod_run, static_cast<void*> (nmpc_),
    &cond_iv_ready_, &mutex_, true
  );

  // Allocate memory
  initial_sd_ = ConstantVector(nmpc_->NXD(), 0);
  initial_pf_ = ConstantVector(nmpc_->NP(), 0);
  initial_qc_ = ConstantVector(nmpc_->NU(), 0);
  final_sd_   = ConstantVector(nmpc_->NXD(), 0);

  hs_ = ConstantVector(mhe_->NXD() + mhe_->NU(), 0);
  ss_ = ConstantVector(mhe_->NXD() + mhe_->NU(), 1);
  ss_ << 1.00, 1.00, 1.00, 1.00, 0.10; // no error

  // @Manuel: from nmpc.cpp
  grl_assert(nmpc_->NU() == action_max_.size());
  grl_assert(nmpc_->NU() == action_min_.size());

  // run single SQP iteration to be able to write a restart file
  nmpc_->feedback();
  nmpc_->transition();
  nmpc_->preparation();

  // Save MUSCOD state
  if (verbose_) {
    std::cout << "saving MUSCOD-II state to" << std::endl;
    std::cout << "  " << nmpc_->m_options->modelDirectory << restart_path_ << "/" << restart_name_ << ".bin" << std::endl;
  }
  nmpc_->m_muscod->writeRestartFile(
    restart_path_.c_str(), restart_name_.c_str()
  );

  if (verbose_)
    std::cout << "MUSCOD-II is ready!" << std::endl;
}

void MHE_NMPCPolicy::reconfigure(const Configuration &config)
{
}


void MHE_NMPCPolicy::muscod_reset(const Vector &initial_obs, const Vector &initial_pf, Vector &initial_qc)
{
  // wait for preparation phase
  if (true) { // TODO Add wait flag
    wait_for_iv_ready(nmpc_, verbose_);
    if (nmpc_->get_iv_ready() == true) {
    } else {
        std::cerr << "MAIN: bailing out ..." << std::endl;
        abort();
    }
  }

  // restore muscod state
  if (verbose_) {
    std::cout << "restoring MUSCOD-II state to" << std::endl;
    std::cout << "  " << nmpc_->m_options->modelDirectory << restart_path_ << "/" << restart_name_ << ".bin" << std::endl;
  }

  nmpc_->m_muscod->readRestartFile(restart_path_.c_str(), restart_name_.c_str());
  nmpc_->m_muscod->nmpcInitialize (
      4,  // 4 for restart
      restart_path_.c_str(), restart_name_.c_str()
  );

  // initialize NMPC
  for (int inmpc = 0; inmpc < 20; ++inmpc)
  {
    // 1) Feedback: Embed parameters and initial value from MHE
    if (initFeedback_) {
      nmpc_->feedback(initial_obs, initial_pf, &initial_qc);
    } else {
      nmpc_->feedback();
    }
    // 2) Transition
    nmpc_->transition();
    // 3) Preparation
    nmpc_->preparation();
  }

  // NOTE: both flags are set to true then iv is provided and
  //       qc is is computed
  // NOTE: due to waiting flag, main thread is on hold until
  //       computations are finished (<=2ms!)
  iv_provided_ = true;
  qc_retrieved_ = true;

  // wait for preparation phase
  if (true) { // TODO Add wait flag
    wait_for_iv_ready(nmpc_, verbose_);
    if (nmpc_->get_iv_ready() == true) {
    } else {
        std::cerr << "MAIN: bailing out ..." << std::endl;
        abort();
    }
  }

  // initialize MHE
  for (int imhe = 0; imhe < 20; ++imhe) {
    // 1) Feedback
    mhe_->feedback();
    // 2) Transition
    mhe_->transition();
    // 3) Preparation
    mhe_->preparation();
  }

  if (verbose_)
    std::cout << "MUSCOD is reseted!" << std::endl;
}

void MHE_NMPCPolicy::act(double time, const Observation &in, Action *out)
{
  grl_assert((in.v.size() == nmpc_->NXD() + nmpc_->NP()) || (in.v.size() == nmpc_->NXD()));

  // subdivide 'in' into state and setpoint
  if (in.size() == nmpc_->NXD() + nmpc_->NP())
  {
    initial_sd_ << in.v.block(0, 0, 1, nmpc_->NXD());
    initial_pf_ << in.v.block(0, nmpc_->NXD(), 1, nmpc_->NP());
  } else {
    initial_sd_ << in.v;
  }

  if (verbose_)
  {
    std::cout << "time: [ " << time << " ]; state: [ " << initial_sd_ << "]" << std::endl;
    std::cout << "                          param: [ " << initial_pf_ << "]" << std::endl;
  }

  if (time == 0.0)
    muscod_reset(initial_sd_, initial_pf_, initial_qc_);

  out->v.resize( nmpc_->NU() );

  if (feedback_ == "non-threaded")
  {
    // Run mutiple MHE iterations
    for (int imhe = 0; imhe < n_iter_; ++imhe) {
      // NOTE compose and inject measurement only at the first iteration of MHE
      if (n_iter_ > 0 && imhe == 0) {
        // 0) Compose new measurement
        // NOTE measurement consists of simulation result + feedback control
        // m_hs = [ xd[0], ..., xd[NXD-1], u[0], ..., u[NU-1] ]
        hs_ << in.v, ConstantVector(mhe_->NU(), 0);
        // std::cout << "new_measurement = " << hs_ << std::endl;
        // 1) Inject measurements
        mhe_->inject_measurement(hs_, ss_, initial_qc_);
        // mhe_->print_horizon();
      }
      // 2) Feedback
      mhe_->feedback();

      // 4) Shifting?
      // NOTE do that only once at last iteration
      // NOTE this has to be done before the transition phase
      if (n_iter_ > 0 && imhe == n_iter_-1) {
        mhe_->shifting(1);
      }

      // 3) Transition
      // NOTE: states and parameters are only updated after transition phase
      mhe_->transition();

      // 3) Get parameters and state of last shooting node
      // NOTE do that only once at last iteration
      if (n_iter_ > 0 && imhe == n_iter_-1) {
        mhe_->get_initial_sd_and_pf(&initial_sd_, &initial_pf_);
      }

      // 6) Preparation
      mhe_->preparation();
    }

    // Run multiple NMPC iterations
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
    out->v[i] = fmax( fmin(initial_qc_[i], action_max_[i]) , action_min_[i]);
    if (out->v[i] != initial_qc_[i])
      WARNING("NMPC action " << i << " was truncated");
  }

  out->type = atGreedy;

  if (verbose_)
    std::cout << "Feedback Control: [" << out->v << "]" << std::endl;
}

