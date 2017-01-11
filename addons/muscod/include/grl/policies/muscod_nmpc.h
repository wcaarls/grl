// *****************************************************************************
// MUSCOD-II NMPC Problem Datau Implementation
//
// A wrapper for MUSCOD NMPC problem data
//
// Copyright (C) 2016, Manuel Kudruss, IWR, Heidelberg
// *****************************************************************************

// *****************************************************************************
// Includes
// *****************************************************************************

// GRL
#include <grl/utils.h>
#include <grl/vector.h>

// C++ standard library
#include <iostream>

// MUSCOD problem data base class
#include "muscod_problem.h"

// MUSCOD-II interface
#include "wrapper.hpp"

// Base class
#include "muscod_problem.h"

// *****************************************************************************
#ifndef MUSCOD_NMPC_H_
#define MUSCOD_NMPC_H_
// *****************************************************************************
// namespace MUSCODData { // BEGIN NAMESPACE MUSCOD
// *****************************************************************************

// *****************************************************************************
// MUSCOD data base class
// *****************************************************************************

struct NMPCProblem : public MUSCODProblem {

  bool m_quit;
  bool m_is_initialized;

  // threading variables
  bool iv_ready_;
  bool qc_ready_;
  bool in_preparation_;

  // NMPC Mode of MUSCOD
  // 0: provide nonlinear feedback with re-linearization each iteration (slowest)
  // 1: provide linear feedback with calling the linearized
  int m_nmpc_mode;

  // Shift mode of NMPC controller
  //  0: shift controls
  //  1: also shift differential and algebraic states
  //  2: also shift evaluated function values
  //  3: also shift evaluated function derivatives
  //  4: also shift evaluated Hessian blocks
  int m_shift_mode;

  // work space for NMPC
  grl::Vector m_sd; // only first shooting node contains initial values
  grl::Vector m_pf; // global parameters
  grl::Vector m_qc; // all controls have to saved

// -----------------------------------------------------------------------------
// thread-safe setters and getters

  int get_nmpc_mode() {
    if (mutex_) {
        int ret;
        pthread_mutex_lock(mutex_);
        ret = m_nmpc_mode;
        pthread_mutex_unlock(mutex_);
        return ret;
    }
    std::cout << "In function '" << __func__ << "' no mutex is assigned!" << std::endl;
    abort();
  }

  void set_nmpc_mode(long mode) {
    if ((mode < 0) || (1 < mode)) {
        std::cout << "Incorrect mode: "<< mode << " not 0 <= mode <= 1!" << std::endl;
        abort();
    }
    if (mutex_) {
        pthread_mutex_lock(mutex_);
        m_nmpc_mode = mode;
        pthread_mutex_unlock(mutex_);
        return;
    }
    std::cout << "In function '" << __func__ << "' no mutex is assigned!" << std::endl;
    abort();
  }

  int get_shift_mode() {
    if (mutex_) {
        int ret;
        pthread_mutex_lock(mutex_);
        ret = m_shift_mode;
        pthread_mutex_unlock(mutex_);
        return ret;
    }
    std::cout << "In function '" << __func__ << "' no mutex is assigned!" << std::endl;
    abort();
  }

  void set_shift_mode(long mode) {
    if ((mode < 0) || (4 < mode)) {
        std::cout << "Incorrect mode: "<< mode << " not 0 <= mode <= 4!" << std::endl;
        abort();
    }
    if (mutex_) {
        pthread_mutex_lock(mutex_);
        m_shift_mode = mode;
        pthread_mutex_unlock(mutex_);
        return;
    }
    std::cout << "In function '" << __func__ << "' no mutex is assigned!" << std::endl;
    abort();
  }


  bool get_iv_ready() {
    if (mutex_) {
        bool ret;
        pthread_mutex_lock(mutex_);
        ret = iv_ready_;
        pthread_mutex_unlock(mutex_);
        return ret;
    }
    std::cout << "In function '" << __func__ << "' no mutex is assigned!" << std::endl;
    abort();
  }

  void set_iv_ready(bool iv_ready) {
    if (mutex_) {
        pthread_mutex_lock(mutex_);
        iv_ready_ = iv_ready;
        pthread_mutex_unlock(mutex_);
        return;
    }
    std::cout << "In function '" << __func__ << "' no mutex is assigned!" << std::endl;
    abort();
  }

  bool get_qc_ready() {
    if (mutex_) {
        bool ret;
        pthread_mutex_lock(mutex_);
        ret = qc_ready_;
        pthread_mutex_unlock(mutex_);
        return ret;
    }
    std::cout << "In function '" << __func__ << "' no mutex is assigned!" << std::endl;
    abort();
  }

  void set_qc_ready(bool qc_ready) {
    if (mutex_) {
        pthread_mutex_lock(mutex_);
        qc_ready_ = qc_ready;
        pthread_mutex_unlock(mutex_);
        return;
    }
    std::cout << "In function '" << __func__ << "' no mutex is assigned!" << std::endl;
    abort();
  }

  bool get_in_preparation () {
    if (mutex_) {
        bool ret;
        pthread_mutex_lock(mutex_);
        ret = in_preparation_;
        pthread_mutex_unlock(mutex_);
        return ret;
    }
    std::cout << "In function '" << __func__ << "' no mutex is assigned!" << std::endl;
    abort();
  }

  void set_in_preparation (bool in_preparation) {
    if (mutex_) {
        pthread_mutex_lock(mutex_);
        in_preparation_ = in_preparation;
        pthread_mutex_unlock(mutex_);
        return;
    }
    std::cout << "In function '" << __func__ << "' no mutex is assigned!" << std::endl;
    abort();
  }

  std::string get_thread_id () {
    if (mutex_) {
        std::string ret;
        pthread_mutex_lock(mutex_);
        ret = thread_id;
        pthread_mutex_unlock(mutex_);
        return ret;
    }
    std::cout << "In function '" << __func__ << "' no mutex is assigned!" << std::endl;
    abort();
  }

// -----------------------------------------------------------------------------

  ~NMPCProblem ()
  {}

  NMPCProblem(
    std::string problem_path, std::string model_name,
    MUSCOD* muscod = NULL
  ) : MUSCODProblem(problem_path, model_name, muscod),
    m_quit (false),
    m_is_initialized (false),
    iv_ready_ (false),
    qc_ready_ (false),
    in_preparation_ (false),
    m_nmpc_mode (0),
    m_shift_mode (1)
 {}

}; // END NMPCProblem

// *****************************************************************************
// } // END NAMESPACE MUSCOD
// *****************************************************************************
// #endif MUSCOD_NMPC_H_
#endif
// *****************************************************************************

