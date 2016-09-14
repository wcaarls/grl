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
  config->push_back(CRP("action_min", "vector.action_min", "Lower limit on actions", action_min_, CRP::System));
  config->push_back(CRP("action_max", "vector.action_max", "Upper limit on actions", action_max_, CRP::System));

  config->push_back(CRP("lua_model", "Lua model used by MUSCOD", lua_model_));
  config->push_back(CRP("model_name", "Name of the model in grl", model_name_));
  config->push_back(CRP("nmpc_model_name", "Name of MUSCOD MHE model library", nmpc_model_name_));
  config->push_back(CRP("outputs", "int.action_dims", "Number of outputs", (int)outputs_, CRP::System, 1));
  config->push_back(CRP("initFeedback", "Initialize feedback", (int)initFeedback_, CRP::System, 0, 1));
  config->push_back(CRP("verbose", "Verbose mode", (int)verbose_, CRP::System, 0, 1));
}

void *NMPCPolicy::setup_model_path(const std::string path, const std::string model, const std::string lua_model)
{
  // get the library handle,
  std::string so_path  = path + "/" + "lib" + model + ".so";
  void *so_handle = dlopen(so_path.c_str(), RTLD_NOW|RTLD_GLOBAL);
  if (so_handle==NULL)
  {
    std::cout << "ERROR: Could not load MUSCOD-II shared model library: '" << so_path << "'" << std::endl;
    std::cout << "dlerror responce: " << dlerror() << std::endl;
    std::cout << "bailing out ..." << std::endl;
    exit(EXIT_FAILURE);
  }

  // get the function handle
  void (*so_set_path)(std::string, std::string);
  std::string so_set_path_fn = "set_path"; // name of a function which sets the path
  so_set_path = (void (*)(std::string, std::string)) dlsym(so_handle, so_set_path_fn.c_str());
  if (so_set_path==NULL)
  {
    std::cout << "ERROR: Could not symbol in shared library: '" << so_set_path_fn << "'" << std::endl;
    std::cout << "bailing out ..." << std::endl;
    std::exit(-1);
  }

  // ... and finally set the paths
  if (verbose_)
  {
    std::cout << "MUSCOD: setting new problem path to: '" << path << "'" <<std::endl;
    std::cout << "MUSCOD: setting new Lua model file to: '" << lua_model << "'" <<std::endl;
  }
  so_set_path(path, lua_model);

  return so_handle;
}

void NMPCPolicy::configure(Configuration &config)
{
  std::string model_path;
  model_path        = std::string(MUSCOD_CONFIG_DIR);
  nmpc_model_name_  = config["nmpc_model_name"].str();
  model_name_       = config["model_name"].str();
  outputs_          = config["outputs"];
  verbose_          = config["verbose"];
  action_min_       = config["action_min"].v();
  action_max_       = config["action_max"].v();

  if (action_min_.size() != action_max_.size())
    throw bad_param("policy/nmpc:{action_min, action_max}");

  // Setup path for the problem description library and lua, csv, dat files used by it
  std::string problem_path  = model_path + "/" + model_name_;

  //-------------------- Load Lua model which is used by muscod ------------------- //
  lua_model_ = problem_path + "/" + config["lua_model"].str();

  struct stat buffer;
  if (stat(lua_model_.c_str(), &buffer) != 0) // check if lua file exists in the problem description folder
    lua_model_ = std::string(RBDL_LUA_CONFIG_DIR) + "/" + config["lua_model"].str(); // if not, then use it as a reference from dynamics

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

