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

  ~NMPCProblem () {}

  NMPCProblem(
    std::string problem_path, std::string model_name,
    MUSCOD* muscod = NULL
  ) : MUSCODProblem(problem_path, model_name, muscod) {

  }

};

// *****************************************************************************
// } // END NAMESPACE MUSCOD
// *****************************************************************************
// #endif MUSCOD_NMPC_H_
#endif
// *****************************************************************************

