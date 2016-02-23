// *****************************************************************************
// Includes
// *****************************************************************************

#include <iostream>
#include <dlfcn.h>
#include <sys/stat.h>

// GRL
#include <grl/policies/mhe_nmpc.h>

// MUSCOD-II interface
#include <wrapper.hpp>

using namespace grl;

REGISTER_CONFIGURABLE(MHE_NMPCPolicy)

MHE_NMPCPolicy::~MHE_NMPCPolicy()
{
  safe_delete(&muscod_mhe_);
  safe_delete(&muscod_nmpc_);
}

void MHE_NMPCPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("lua_model", "Lua model used by MUSCOD", lua_model_));
  config->push_back(CRP("mhe_model_name", "Name of MUSCOD MHE model library", mhe_model_name_));
  config->push_back(CRP("nmpc_model_name", "Name of MUSCOD MHE model library", nmpc_model_name_));
  config->push_back(CRP("outputs", "int.action_dims", "Number of outputs", (int)outputs_, CRP::System, 1));
  config->push_back(CRP("verbose", "Verbose mode", (int)verbose_, CRP::System, 0, 1));
}

void *MHE_NMPCPolicy::setup_model_path(const std::string path, const std::string model, const std::string lua_model)
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

void MHE_NMPCPolicy::configure(Configuration &config)
{
  std::string model_path;
  model_path        = std::string(MUSCOD_CONFIG_DIR);
  mhe_model_name_   = config["mhe_model_name"].str();
  nmpc_model_name_  = config["nmpc_model_name"].str();
  outputs_          = config["outputs"];
  verbose_          = config["verbose"];

  // Setup path for the problem description library and lua, csv, dat files used by it
  std::string problem_path  = model_path + "/" + mhe_model_name_ + "_" + nmpc_model_name_;

  //-------------------- Load Lua model which is used by muscod ------------------- //
  lua_model_ = problem_path + "/" + config["lua_model"].str();

  struct stat buffer;
  if (stat(lua_model_.c_str(), &buffer) != 0) // check if lua file exists in the problem description folder
    lua_model_ = std::string(RBDL_LUA_CONFIG_DIR) + "/" + config["lua_model"].str(); // if not, then use it as a reference from dynamics

  //----------------- Set path in the problem description library ----------------- //
  setup_model_path(problem_path, mhe_model_name_, lua_model_);
  void * so_handle_nmpc = setup_model_path(problem_path, nmpc_model_name_, lua_model_);

  //----------------- Observation converter ----------------- //
  so_convert_obs_for_muscod_ = (t_obs_converter) dlsym(so_handle_nmpc, "convert_obs_for_muscod");
  if (so_convert_obs_for_muscod_ == NULL)
  {
    std::cout << "ERROR: Could not symbol in shared library: 'convert_obs_for_muscod'" << std::endl;
    std::cout << "bailing out ..." << std::endl;
    std::exit(-1);
  }

  //------------------- Initialize MHE ------------------- //
  muscod_mhe_ = new MUSCOD();
  mhe_ = new MHEProblem(problem_path.c_str(), mhe_model_name_.c_str(), muscod_mhe_);

  Eigen::VectorXd hs(mhe_->NXD() + mhe_->NU()), ss(mhe_->NXD() + mhe_->NU()), qs(mhe_->NU());
  hs << 0.00, 3.14, 0.00, 0.00, 0.00; // hanging down
  ss << 1.00, 1.00, 1.00, 1.00, 1.00; // no error
  qs << 0.00; // no control

  std::cout << hs << std::endl;
  std::cout << ss << std::endl;
  std::cout << qs << std::endl;

  // initialize measurement horizon with data
  mhe_->initialize_horizon(hs, ss, qs);

  //------------------- Initialize NMPC ------------------- //
  muscod_nmpc_ = new MUSCOD();
  nmpc_ = new NMPCProblem(problem_path.c_str(), nmpc_model_name_.c_str(), muscod_nmpc_);

  // Allocate memory
  real_pf_ = VectorConstructorFill(nmpc_->NP(), 0);
  real_pf_(0) = 0.06;

  initial_sd_ = VectorConstructorFill(nmpc_->NXD(), 0);
  initial_pf_ = VectorConstructorFill(nmpc_->NP(), 0);
  initial_qc_ = VectorConstructorFill(nmpc_->NU(), 0);
  final_sd_   = VectorConstructorFill(nmpc_->NXD(), 0);

  hs_ = VectorConstructorFill(mhe_->NXD() + mhe_->NU(), 0);
  ss_ = VectorConstructorFill(mhe_->NXD() + mhe_->NU(), 1);

/*  muscod_->setModelPathAndName(problem_path.c_str(), model_name_.c_str());
  muscod_->loadFromDatFile(".", model_name_.c_str());
  muscod_->setLogLevelScreen(-1);
  muscod_->setLogLevelAndFile(-1, NULL, NULL);

  // get proper dimensions
  muscod_->nmpcInitialize(muscod_->getSSpec(), NULL, NULL);

  // solve until convergence to prepare solver
  for (int ii=0; ii < 50; ++ii)
  {
    muscod_->nmpcFeedback(NULL, NULL, NULL);
    muscod_->nmpcTransition();
    muscod_->nmpcPrepare();
  }

  // save solver state
  data_.backup_muscod_state(muscod_);
  data_.sd = ConstantVector(data_.NXD, 0.0);
  data_.pf = ConstantVector(data_.NP,  0.0);
*/

  if (verbose_)
    std::cout << "MUSCOD is ready!" << std::endl;
}

void MHE_NMPCPolicy::reconfigure(const Configuration &config)
{
}


void MHE_NMPCPolicy::muscod_reset(Vector &initial_obs, double time)
{
/*
  // load solution state
  data_.restore_muscod_state(muscod_);

  // Reinitialize state and time
  for (int IP = 0; IP < data_.NP; ++IP)
    data_.pf[IP] = time;

  // solve until convergence to prepare solver
  for (int ii=0; ii < 50; ++ii)
  {
    muscod_->nmpcFeedback(initial_obs.data(), data_.pf.data(), NULL);
    muscod_->nmpcTransition();
    muscod_->nmpcPrepare();
  }
*/
  if (verbose_)
    std::cout << "MUSCOD is reseted!" << std::endl;
}

MHE_NMPCPolicy *MHE_NMPCPolicy::clone() const
{
  return NULL;
}

void MHE_NMPCPolicy::act(double time, const Vector &in, Vector *out)
{
  if (verbose_)
    std::cout << "observation state: [ " << in << "]" << std::endl;

  Vector obs;
  obs.resize(in.size());
  if (time == 0.0)
  {
    so_convert_obs_for_muscod_(NULL, NULL);            // Reset internal counters
    so_convert_obs_for_muscod_(in.data(), obs.data()); // Convert
    muscod_reset(obs, time);
  }

  // Convert MPRL states into MUSCOD states
  so_convert_obs_for_muscod_(in.data(), obs.data());

  if (verbose_)
    std::cout << "time: [ " << time << " ]; state: [ " << obs << "]" << std::endl;

  out->resize(outputs_);
//  for (int IP = 0; IP < data_.NP; ++IP)
//    data_.pf[IP] = time;

  for (int ii = 0; ii < 10; ++ii)
  {
    // NMPC
    // 1) Feedback: Embed parameters and initial value from MHE
    std::cout << "FEEDBACK" << std::endl;
    nmpc_->feedback(initial_sd_, initial_pf_, &initial_qc_);
    // 2) Transition
    std::cout << "TRANSITION" << std::endl;
    nmpc_->transition();
    // 3) Shifting
    std::cout << "SHIFTING" << std::endl;
    nmpc_->shifting(1);
    // 4) Preparation
    std::cout << "PREPARATION" << std::endl;
    nmpc_->preparation();

    // Simulate
    // TODO collect runtime of algorithms and add to simulation time
    // 1) Get new initial value applying real parameters and NMPC control
    nmpc_->simulate(
      initial_sd_,
      real_pf_,
      initial_qc_,
      nmpc_->getSamplingRate(),
      &final_sd_
    );

    // MHE
    // 1) Inject measurements
    std::cout << "INJECT MEASUREMENT" << std::endl;
    mhe_->inject_measurement(hs_, ss_, initial_qc_);
    mhe_->print_horizon();
    // 2) Feedback
    std::cout << "FEEDBACK" << std::endl;
    mhe_->feedback();
    // 3) Transition
    std::cout << "TRANSITION" << std::endl;
    mhe_->transition();
    // 4) Shifting?
    std::cout << "SHIFTING" << std::endl;
    mhe_->shifting(1);
    // 5) Preparation
    std::cout << "PREPARATION" << std::endl;
    mhe_->preparation();
  }

  if (verbose_)
    std::cout << "Feedback Control: [" << *out << "]" << std::endl;
}

