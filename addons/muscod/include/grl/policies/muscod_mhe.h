// *****************************************************************************
// MUSCOD-II MHE Problem Datau Implementation
//
// A wrapper for MUSCOD MHE problem data
//
// Copyright (C) 2016, Manuel Kudruss, IWR, Heidelberg
// *****************************************************************************

// *****************************************************************************
// Includes
// *****************************************************************************

// C++ standard library
#include <iostream>

// MUSCOD problem data base class
#include "muscod_problem.h"

// MUSCOD-II interface
#include "wrapper.hpp"

// Base class
#include "muscod_problem.h"

// *****************************************************************************
#ifndef MUSCOD_MHE_H_
#define MUSCOD_MHE_H_
// *****************************************************************************
// namespace MUSCODData { // BEGIN NAMESPACE MUSCOD
// *****************************************************************************

// *****************************************************************************
// MUSCOD data base class
// *****************************************************************************

struct MHEProblem : public MUSCODProblem {

  // reference to the problem library measurement horizon
  Eigen::MatrixXd* m_meas_hs;
  Eigen::MatrixXd* m_meas_ss;

  // shared library handles
  void (*so_print_horizon) ();
  Eigen::MatrixXd* (*so_get_measurements)();
  Eigen::MatrixXd* (*so_get_sigmas)();

  void print_horizon(){
    so_print_horizon();
  }

  void initialize_horizon(grl::Vector& hs, grl::Vector& ss, grl::Vector qs){
    for (int imsn = 0; imsn < m_NMSN-1; ++imsn) {
      (*m_meas_hs).col(imsn) = hs;
      (*m_meas_ss).col(imsn) = ss;
        m_muscod->setNodeQC(imsn, qs.data());
    }
    // TODO fix controls
  }

  void inject_measurement(
    const Eigen::VectorXd& hs,
    const Eigen::VectorXd& ss,
    const Eigen::VectorXd& qs
  ) {
    // shift horizon
    long int n = (*m_meas_hs).cols() - 1;
    // m_ts.head(n) = m_ts.tail(n);
    // NOTE: m_meas_xx.leftCols(m_meas_xx.cols()-1) := m_meas_xx[:-1]
    //       m_meas_xx.rightCols(m_meas_xx.cols()-1) := m_meas_xx[1:]
    (*m_meas_hs).leftCols(n) = (*m_meas_hs).rightCols(n);
    (*m_meas_ss).leftCols(n) = (*m_meas_ss).rightCols(n);

    // write new values in rightmost column
    // std::cout << (*m_meas_hs).rightCols(1).cols() << " = " << hs.cols() << std::endl;
    // std::cout << (*m_meas_hs).rightCols(1).rows() << " = " << hs.rows() << std::endl;
    (*m_meas_hs).rightCols(1) = hs;
    (*m_meas_ss).rightCols(1) = ss;
  }

  // ---------------------------------------------------------------------------

  ~MHEProblem ()
  {
    // close shared library
    so_print_horizon = NULL;
    so_get_measurements = NULL;
    so_get_sigmas = NULL;

    std::string so_path  = m_problem_path + "/" + "lib" + m_model_name + ".so";
    if (m_verbose) {
      std::cout << "unloading shared library";
      std::cout << so_path << "'" << std::endl;
    }
    dlclose(m_problem_so_handle);
  }

  MHEProblem(
    std::string problem_path, std::string model_name,
    MUSCOD* muscod = NULL
  ) : MUSCODProblem(problem_path, model_name, muscod)
  {
    std::string so_path  = m_problem_path + "/" + "lib" + m_model_name + ".so";
    m_problem_so_handle = load_so(so_path);

    so_print_horizon = (void (*) ())
      load_symbol(m_problem_so_handle, "print_horizon");

    so_get_measurements = (Eigen::MatrixXd* (*) ())
      load_symbol(m_problem_so_handle, "get_measurements");

    so_get_sigmas = (Eigen::MatrixXd* (*) ())
      load_symbol(m_problem_so_handle, "get_sigmas");

    m_meas_hs = so_get_measurements ();
    m_meas_ss = so_get_sigmas ();
  }

};

// *****************************************************************************
// } // END NAMESPACE MUSCOD
// *****************************************************************************
// #endif MUSCOD_MHE_H_
#endif
// *****************************************************************************

