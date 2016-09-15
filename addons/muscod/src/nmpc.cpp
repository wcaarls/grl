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

REGISTER_CONFIGURABLE(NMPCPolicy);

double stop_watch(){
    static struct timespec timer;
    if (clock_gettime (CLOCK_REALTIME, &timer) != 0) return -1;
    return (double) timer.tv_sec + 1.0E-9 * (double) timer.tv_nsec;
}

// clean-up MUSCOD-II main thread
void stop_thread (
    NMPCProblem& data_,
    pthread_t* muscod_thread_,
    bool verbose = false
) {
    bool wait = true;
    unsigned int cnt;
    if (wait) {
        cnt = 0;

        // debug message
        if (verbose) {
            std::cout << "MAIN: Waiting for thread to get ready!" << std::endl;
        }

        // wait until thread is ready, i.e. cond_iv_ready_ condition is established
        while (!data_.get_iv_ready()) {
            if (cnt > 10000) {
                if (verbose) {
                    std::cerr << "MAIN: thread frozen!" << std::endl;
                }
                break;
            }
            usleep(100); // wait for 0.1 ms
            cnt += 1;
        }

        if (verbose) {
            std::cout << "MAIN: Waiting for " << (double) cnt * 0.1;
            std::cout << "ms for thread!" << std::endl;
        }

        // assure thread is ready
        if (data_.get_iv_ready() == false) {
            std::cerr << "MAIN: bailing out ..." << std::endl;
            abort();
        }
    }

    //release the thread from waiting if needed
    pthread_mutex_lock(data_.mutex_);
    data_.m_quit = true; // send quit signal
    pthread_cond_signal(data_.cond_iv_ready_); // send cond_iv_ready signal
    pthread_mutex_unlock(data_.mutex_);

    // join thread, which closes the thread
    pthread_join(*muscod_thread_, NULL);
}

NMPCPolicy::~NMPCPolicy()
{
  // stop threads
  stop_thread (*nmpc_, &thread_);

  // safely delete instances
  safe_delete(&nmpc_);
  safe_delete(&muscod_);
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

// run MUSCOD-II NMPC for several iterations to initialize controller for
// current initial sd/pf, first qc is optional
void initialize_controller (
    MUSCODProblem& nmpc, const int nmpc_ninit_,
    Vector& sd, Vector& pf,
    Vector* qc = NULL
) {
    // initialize NMPC by nmpc_init SQP iterations
    for (int inmpc = 0; inmpc < nmpc_ninit_; ++inmpc) {
        // 1) Feedback: Embed parameters and initial value
        nmpc.feedback(sd, pf, qc);
        // 2) Transition
        nmpc.transition();
        // 3) Preparation
        nmpc.preparation();
    }
}

// Initialize mutex and condition variable objects and the controller thread
void initialize_thread(
    pthread_t* muscod_thread_,
    void* (*function) (void*) ,
    void* data,
    pthread_cond_t*  cond_iv_ready_,
    pthread_mutex_t* mutex_,
    bool verbose = false
) {
    // initialize mutex and condition variable
    pthread_mutex_init(mutex_, NULL);
    pthread_cond_init (cond_iv_ready_, NULL);

    // start MUSCOD-II in a thread running a signal triggered execution loop
    pthread_mutex_lock(mutex_); // LOCK
    if (verbose) {
        std::cout << "In " << __func__ << ": creating MUSCOD-II thread" << std::endl;
    }

    // create thread running execution loop
    int rc = pthread_create (
        muscod_thread_, NULL, function, data
    );

    // error message on error (rc > 0 if error happened!)
    if (rc) {
      std::cerr << "ERROR: pthread_create() failed with " << rc << std::endl;
      std::cerr << "bailing out..." << rc << std::endl;
      abort();
    }

    // wait for MUSCOD thread to initialize data structure
    if (verbose) {
        std::cout << "MUSCOD: Waiting for MUSCOD thread..." << std::endl;
    }
    pthread_cond_wait(cond_iv_ready_, mutex_); // WAIT FOR SIGNAL FROM THREAD
    pthread_mutex_unlock(mutex_);  // UNLOCK
}

// MUSCOD-II main thread setup and execution loop
void *muscod_run (void *indata)
{
    // timing variables
    double tic, tac, ttac, ptac;

    // cast data back to object
    if (indata == NULL) {
        std::cerr << "ERROR: indata is NULL" << std::endl;
        std::cerr << "bailing out..." << std::endl;
        abort();
    }
    NMPCProblem& nmpc = *static_cast<NMPCProblem*> (indata);

    // retrieve verbose flag from controller
    pthread_mutex_lock(nmpc.mutex_);
    bool verbose = nmpc.m_verbose;
    pthread_mutex_unlock(nmpc.mutex_);

    // retrieve thread identifier from NMPC instance
    pthread_mutex_lock(nmpc.mutex_);
    std::string thread_id = nmpc.thread_id;
    pthread_mutex_unlock(nmpc.mutex_);

    if (!thread_id.empty()) {
        if (verbose) {
            std::cout << "THREAD '" << thread_id << "': got thread id! " << std::endl;
        }
    }

    // instantiate values of NMPC structure
    pthread_mutex_lock(nmpc.mutex_);
    unsigned long NMSN = nmpc.NMSN();
    unsigned long NXD = nmpc.NXD();
    unsigned long NP = nmpc.NP();
    unsigned long NU = nmpc.NU();

    // define initial value and placeholder for feedback
    Vector sd = Vector::Zero (NXD);
    Vector pf = Vector::Zero (NP);
    Vector qc = Vector::Zero (NU);

    // same for exchange TODO: move this to backup_muscod_state
    nmpc.m_sd = Vector::Zero (NXD);
    nmpc.m_pf = Vector::Zero (NP);
    nmpc.m_qc = Vector::Zero (NU);
    nmpc.m_is_initialized = true;

    // release setup of thread
    pthread_cond_signal(nmpc.cond_iv_ready_);
    pthread_mutex_unlock(nmpc.mutex_);

    if (verbose) {
        std::cout << "THREAD '" << thread_id << "': MUSCOD thread is ready!" << std::endl;
    }

    // EXECUTION LOOP
    while (true) {
        pthread_mutex_lock(nmpc.mutex_); // --> Lock the mutex
        // LOCK THE MUTEX
        // NOTE: wait for cond_iv_ready_ signal and then lock the mutex again
        nmpc.iv_ready_ = true;
        nmpc.in_preparation_ = false;
        pthread_cond_wait(nmpc.cond_iv_ready_, nmpc.mutex_);

        // reset timing statistics
        nmpc.timing.reset ();

        // LEAVE THREAD IF QUIT FLAG IS SET IN DATA
        if (nmpc.m_quit) {
            pthread_mutex_unlock(nmpc.mutex_);
            break; // exit when thread is blocked
        }

        // GET INITIAL VALUES AND PARAMETERS FROM DATA
        sd << nmpc.m_sd;
        pf << nmpc.m_pf;
        if (verbose) {
            std::cout << "THREAD '" << thread_id << "': sd = ";
            std::cout << sd.transpose () << std::endl;
            std::cout << "THREAD '" << thread_id << "': pf = ";
            std::cout << pf.transpose () << std::endl;
        }

        // UNLOCK THE MUTEX
        pthread_mutex_unlock(nmpc.mutex_);

        // NMPC loop
        tic = stop_watch(); // <- timing execution
        ptac = 0.0;
        ttac = 0.0;

        // 1) Feedback: Embed parameters and initial value from SIMULATION
        nmpc.feedback(sd, pf, &qc);

        tac = stop_watch(); // <- timing execution
        ttac += tac - tic;
        pthread_mutex_lock(nmpc.mutex_); // --> Lock the mutex
        nmpc.timing.time_feedback(tac - tic);
        pthread_mutex_unlock(nmpc.mutex_); // --> Unlock the mutex

        if (verbose) {
            std::cout << "THREAD '" << thread_id << "': qc = " << qc.transpose() << std::endl;
        }

        pthread_mutex_lock(nmpc.mutex_); // --> Lock the mutex
        // LEAVE THREAD IF QUIT FLAG IS SET IN DATA
        if (nmpc.m_quit) {
            pthread_mutex_unlock(nmpc.mutex_);
            break; // exit when thread is blocked
        }

        // PUT CONTROLS INTO NMPC STRUCTURE
        // NOTE: Let main thread know that controls are ready
        //       and thread requires new state!
        nmpc.m_qc << qc;

        // signal to MUSCOD thread to embed initial values and start the computation
        nmpc.qc_ready_ = true;
        if (verbose) {
            std::cout << "THREAD '" << thread_id << "': Signaling 'QC ready'!" << std::endl;
        }

        // signal back that preparation phase has started
        nmpc.in_preparation_ = false;

        // UNLOCK THE MUTEX
        pthread_mutex_unlock(nmpc.mutex_);

        // when in NMPC mode then continue re-linearization
        if (nmpc.m_nmpc_mode == 0) {
            if (verbose) {
                std::cout << "THREAD '" << thread_id << "': MODE 0" << std::endl;
            }
            // 2) Transition
            int shift = nmpc.get_shift_mode();

            // 3) Shifting
            tic = stop_watch(); // <- time execution
            nmpc.shifting(shift);
            tac = stop_watch();
            ptac += tac - tic;
            ttac += tac - tic;
            pthread_mutex_lock(nmpc.mutex_); // --> Lock the mutex
            nmpc.timing.time_shift(tac - tic);
            pthread_mutex_unlock(nmpc.mutex_); // --> Unlock the mutex

            // 4) Transition
            tic = stop_watch();
            nmpc.transition();
            tac = stop_watch();
            ptac += tac - tic;
            ttac += tac - tic;
            pthread_mutex_lock(nmpc.mutex_); // --> Lock the mutex
            nmpc.timing.time_transition(tac - tic);
            pthread_mutex_unlock(nmpc.mutex_); // --> Unlock the mutex

            // 4) Preparation
            tic = stop_watch();
            nmpc.preparation();
            tac = stop_watch();
            ptac += tac - tic;
            ttac += tac - tic;
            pthread_mutex_lock(nmpc.mutex_); // --> Lock the mutex
            nmpc.timing.time_prepare(tac - tic);
            pthread_mutex_unlock(nmpc.mutex_); // --> Unlock the mutex
        } // END IF NMPC MODE

        // time total evaluation time of NMPC iteration
        pthread_mutex_lock(nmpc.mutex_); // --> Lock the mutex
        nmpc.timing.time_preparation(ptac);
        nmpc.timing.time_total(ttac);
        pthread_mutex_unlock(nmpc.mutex_); // --> Unlock the mutex
    } // END WHILE LOOP

    if (verbose) {
        std::cout << "THREAD '" << thread_id << "': exiting MUSCOD thread" << std::endl;
    }
    pthread_exit(NULL);
} // END OF muscod_run

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
  muscod_ = new MUSCOD();
  if (verbose_) {
    muscod_->setLogLevelAndFile(-1, NULL, NULL);
  } else {
    muscod_->setLogLevelTotal(-1);
  }
  nmpc_ = new NMPCProblem(problem_path.c_str(), nmpc_model_name_.c_str(), muscod_);
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
  //initial_sd_ = ConstantVector(nmpc_->NXD(), 0);
  initial_pf_ = ConstantVector(nmpc_->NP(), 0);
  initial_qc_ = ConstantVector(nmpc_->NU(), 0);
  initial_sd_ = ConstantVector(nmpc_->NXD(), 0);
  final_sd_   = ConstantVector(nmpc_->NXD(), 0);

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

void wait_for_iv_ready (NMPCProblem* nmpc, bool verbose = false)
{
    unsigned long cnt = 0;
    double total_time = 0.0;

    if (verbose) {
        std::cout << "MAIN: Waiting for thread to get ready!" << std::endl;
    }

    // sleep loop until thread is ready
    while (!nmpc->get_iv_ready()) {
        if (cnt >= 10000) {
            if (verbose) {
                std::cerr << "MAIN: thread frozen!" << std::endl;
            }
            break;
        }
        usleep(100); // wait for 0.1 ms
        cnt += 1;
    }

    // compute total_time
    total_time = (double) cnt * 0.1; // in ms

    if (verbose) {
        std::cout << "MAIN: Waiting for " << total_time;
        std::cout << "ms for preparation!" << std::endl;
    }

    if (nmpc->get_iv_ready() == false) {
        std::cerr << "MAIN: bailing out ..." << std::endl;
        abort();
    }
}

void provide_iv (
    NMPCProblem* nmpc,
    const Vector& initial_sd,
    const Vector& initial_pf,
    bool* iv_provided,
    bool wait = true,
    bool verbose = false
) {
    // want to provide initial values
    if ( !(*iv_provided) ) {
        std::cerr << "MAIN: in " << __func__ << " ..." << std::endl;
        std::cerr << "MAIN: iv_provided = " << *iv_provided << std::endl;
        std::cerr << "MAIN: skipped!" << std::endl;
        return;
    }

    // wait for thread to finish computations
    if (wait) {
        wait_for_iv_ready(nmpc, verbose);
    } // endif wait

    // PROVIDE INITIAL VALUES WHEN CONTROLLER IS READY
    if (nmpc->get_iv_ready() == true) {
          if (verbose) {
              std::cout << "MAIN: iv_ready: " << (bool) nmpc->get_iv_ready();
              std::cout << std::endl;
          }

          // LOCK THREAD
          pthread_mutex_lock(nmpc->mutex_);

          // Provide state and parameters
          nmpc->m_sd << initial_sd;
          nmpc->m_pf << initial_pf;

          // signal to MUSCOD thread to embed initial values and start the
          // computation
          nmpc->iv_ready_ = false;
          pthread_cond_signal(nmpc->cond_iv_ready_);

          // UNLOCK THREAD
          pthread_mutex_unlock(nmpc->mutex_);

          // signal back that initial values was provided
          *iv_provided = true;
    }

    return;
}

void wait_for_qc_ready (NMPCProblem* nmpc, bool verbose = false)
{
    unsigned long cnt = 0;
    double total_time = 0.0;

    if (verbose) {
        std::cout << "MAIN: Waiting for thread to finish calculations!" << std::endl;
    }

    while (!nmpc->get_qc_ready()) {
        if (cnt >= 10000) {
            if (verbose) {
                std::cerr << "MAIN: thread frozen!" << std::endl;
            }
            break;
        }
        usleep(100); // wait for 0.1 ms
        cnt += 1;
    }

    // compute total_time
    total_time = (double) cnt * 0.1; // in ms

    if (verbose) {
        std::cout << "MAIN: Waiting for " << total_time;
        std::cout << "ms for feedback!" << std::endl;
    }

    if (nmpc->get_qc_ready() == false) {
        std::cerr << "MAIN: bailing out ..." << std::endl;
        abort();
    }
}

void retrieve_qc (
    NMPCProblem* nmpc,
    Vector* first_qc,
    bool* qc_retrieved,
    bool wait = true,
    bool verbose = false
) {
    double total_time = 0.0;
    unsigned long cnt;

    // want to retrieve feedback?
    if ( !(*qc_retrieved) ) {
        std::cerr << "MAIN: in " << __func__ << " ..." << std::endl;
        std::cerr << "MAIN: qc_retrieved = " << *qc_retrieved << std::endl;
        std::cerr << "MAIN: skipped!" << std::endl;
        return;
    }

    // wait for thread to finish computations
    if (wait) {
        wait_for_qc_ready(nmpc, verbose);
    } // endif wait

    // RETRIEVE FEEDBACK CONTROLS WHEN COMPUTATIONS ARE FINISHED
    if (nmpc->get_qc_ready() == true) {
        if (verbose) {
            std::cout << "MAIN: qc_ready: " << (bool) nmpc->get_qc_ready() << std::endl;
        }
        // LOCK THREAD
        pthread_mutex_lock(nmpc->mutex_);

        // Retrieve feedback control from controller
        *first_qc << nmpc->m_qc;

        // revoke ready flag in thread
        nmpc->qc_ready_ = false;

        // UNLOCK THREAD
        pthread_mutex_unlock(nmpc->mutex_);

        if (verbose) {
            std::cout << "MAIN: qc = " << first_qc->transpose() << std::endl;
        }

        // signal back that control was provided
        *qc_retrieved = true;
    }

    return;
}

// handle communication with MUSCOD thread
// INPUT: iv_provided = false => skip providing iv
// INPUT: qc_retrieved = false => skip retrieving qc
// INPUT: wait = true => wait for MUSCOD-II thread to be finished
// return value is total time

void get_feedback (
    NMPCProblem* nmpc,
    const Vector& initial_sd,
    const Vector& initial_pf,
    Vector* first_qc,
    bool* iv_provided,
    bool* qc_retrieved,
    bool wait = true,
    bool verbose = false
) {
    // provide iv to thread, get time until thread was ready
    provide_iv (nmpc, initial_sd, initial_pf, iv_provided, wait, verbose);

    // provide iv to thread, get time until thread was ready
    retrieve_qc (nmpc, first_qc, qc_retrieved, wait, verbose);

    return;
}

TransitionType NMPCPolicy::act(double time, const Vector &in, Vector *out)
{
  grl_assert(in.size() == nmpc_->NXD() + nmpc_->NP()); // setpoint indicator
  grl_assert(outputs_  == nmpc_->NU());

  // reference height
  initial_pf_ << in[in.size()-1];

  // remove indicator
  initial_sd_ << in.block(0, 0, 1, in.size()-1);

  if (verbose_)
  {
    std::cout << "time: [ " << time << " ]; state: [ " << initial_sd_ << "]" << std::endl;
    std::cout << "                          param: [ " << initial_pf_ << "]" << std::endl;
  }

  if (time == 0.0)
    muscod_reset(initial_sd_, initial_pf_, initial_qc_);

  out->resize(outputs_);

   // Run multiple NMPC iterations
  const unsigned int nnmpc = 0;
  for (int inmpc = 0; inmpc < nnmpc; ++inmpc) {
    std::cout << "NON-THREADED VERSION!" << std::endl;
    // 1) Feedback: Embed parameters and initial value from MHE
    // NOTE the same initial values (sd, pf) are embedded several time,
    //      but this will result in the same solution as running a MUSCOD
    //      instance for several iterations
    nmpc_->feedback(initial_sd_, initial_pf_, &initial_qc_);
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
  // } // END FOR NMPC ITERATIONS

   // Run multiple NMPC iterations
  const unsigned int nnmpc_ = 1;
  for (int inmpc = 0; inmpc < nnmpc_; ++inmpc) {
      // std::cout << "THREADED VERSION!" << std::endl;
      // 1) Feedback: Embed parameters and initial value from SIMULATION
      // establish IPC communication to NMPC thread

      // NOTE: both flags are set to true then iv is provided and
      //       qc is is computed
      // NOTE: due to waiting flag, main thread is on hold until
      //       computations are finished (<=2ms!)
      iv_provided_ = true;
      qc_retrieved_ = true;

      // establish IPC communication to NMPC thread
      get_feedback (
          nmpc_,
          initial_sd_,
          initial_pf_,
          &initial_qc_,
          &iv_provided_,
          &qc_retrieved_,
          // NOTE: we use wait flag here to guarantee separation of
          //       feedback phases
          // TODO do something with it
          true // wait flag
      );

      // wait for preparation phase
      if (true) { // TODO Add wait flag
        wait_for_iv_ready(nmpc_, verbose_);
        if (nmpc_->get_iv_ready() == true) {
        } else {
            std::cerr << "MAIN: bailing out ..." << std::endl;
            abort();
        }
      }
  } // END FOR NMPC ITERATIONS

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

