// *****************************************************************************
// Includes
// *****************************************************************************

#include <iostream>
#include <dlfcn.h>
#include <sys/stat.h>
#include <iomanip>

// GRL
#include <grl/policies/nmpc.h>

// MUSCOD-II interface
#include <wrapper.hpp>

using namespace grl;

REGISTER_CONFIGURABLE(NMPCPolicy)

NMPCPolicy::~NMPCPolicy()
{
  safe_delete(&muscod_nmpc_);
}

void NMPCPolicy::request(ConfigurationRequest *config)
{
  NMPCBase::request(config);
}

void NMPCPolicy::configure(Configuration &config)
{
  NMPCBase::configure(config);

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
  setup_model_path(problem_path, nmpc_model_name_, lua_model_);

  //------------------- Initialize NMPC ------------------- //
  muscod_nmpc_ = new MUSCOD();
  if (verbose_) {
    muscod_nmpc_->setLogLevelAndFile(-1, NULL, NULL);
  } else {
    muscod_nmpc_->setLogLevelTotal(-1);
  }
  nmpc_ = new NMPCProblem(problem_path.c_str(), nmpc_model_name_.c_str(), muscod_nmpc_);

  // Allocate memory
  //initial_sd_ = ConstantVector(nmpc_->NXD(), 0);
  initial_pf_ = ConstantVector(nmpc_->NP(), 0);
  initial_qc_ = ConstantVector(nmpc_->NU(), 0);
  final_sd_   = ConstantVector(nmpc_->NXD(), 0);

  // Muscod params
  initFeedback_ = config["initFeedback"];

  nmpc_->feedback();
  nmpc_->transition();
  nmpc_->preparation();

  // Save muscod state
  if (verbose_) {
    std::cout << "saving MUSCOD-II state to" << std::endl;
    std::cout << "  " << nmpc_->m_options->modelDirectory << restart_path_ << "/" << restart_name_ << ".bin" << std::endl;
  }
  nmpc_->m_muscod->writeRestartFile(
    restart_path_.c_str(), restart_name_.c_str()
  );

  if (verbose_)
    std::cout << "MUSCOD is ready!" << std::endl;
}

void NMPCPolicy::reconfigure(const Configuration &config)
{
}

void NMPCPolicy::muscod_reset(const Vector &initial_obs, const Vector &initial_pf, Vector &initial_qc) const
{
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

  if (verbose_)
    std::cout << "MUSCOD is reseted!" << std::endl;
}

NMPCPolicy *NMPCPolicy::clone() const
{
  return NULL;
}

TransitionType NMPCPolicy::act(double time, const Vector &in, Vector *out)
{
  grl_assert(in.size() == nmpc_->NXD() + 1); // setpoint indicator
  grl_assert(outputs_  == nmpc_->NU());

  // reference height
  initial_pf_ << in[in.size()-1];

  // remove indicator
  Vector in2 = in.block(0, 0, 1, in.size()-1);

  if (verbose_)
  {
    std::cout << "time: [ " << time << " ]; state: [ " << in2 << "]" << std::endl;
    std::cout << "                          param: [ " << initial_pf_ << "]" << std::endl;
  }

  if (time == 0.0)
    muscod_reset(in2, initial_pf_, initial_qc_);

  out->resize(outputs_);

  // simulate model over specified time interval using NMPC internal model
  if (verbose_)
  {
    double time_interval = 0.03; //nmpc_->getSamplingRate();
    nmpc_->simulate(
        in2,
        initial_pf_,
        initial_qc_,
        time_interval,
        &final_sd_
    );
    std::cout << "NMPC simulation result using time_interval = " << time_interval << " is:" << std::endl;
    std::cout << final_sd_ << std::endl;
  }

  // Run multiple NMPC iterations
  const unsigned int nnmpc = 1;
  for (int inmpc = 0; inmpc < nnmpc; ++inmpc) {
    // 1) Feedback: Embed parameters and initial value from MHE
    // NOTE the same initial values (sd, pf) are embedded several time,
    //      but this will result in the same solution as running a MUSCOD
    //      instance for several iterations
    nmpc_->feedback(in2, initial_pf_, &initial_qc_);
    // 2) Shifting
    // NOTE do that only once at last iteration
    // NOTE this has to be done before the transition phase
    if (nnmpc > 0 && inmpc == nnmpc-1) {
      nmpc_->shifting(1);
    }
    // 3) Transition
    nmpc_->transition();
    // 4) Preparation
    nmpc_->preparation();
  }

  // Here we can return the feedback control
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

