// *****************************************************************************
// MUSCOD-II RBDL Model Wrapper
//
// RBDL model wrapper for MUSCOD application
//
// Copyright (C) 2015, Manuel Kudruss, IWR, Heidelberg
// *****************************************************************************

// *****************************************************************************
// Includes
// *****************************************************************************

// RBDL dependencies
#include "rbdl/rbdl.h"
#include <rbdl/addons/luamodel/luamodel.h>

// *****************************************************************************
#ifndef RBDL_MODEL_H_
#define RBDL_MODEL_H_
// *****************************************************************************
// namespace CommonCode { // BEGIN NAMESPACE CommonCode
// *****************************************************************************

// *****************************************************************************
// Model
// *****************************************************************************

struct RBDLModel {

    bool is_initialized;

    // RBDL variables
    unsigned int ndof;
    unsigned int nadof;

    RigidBodyDynamics::Model model;
    RigidBodyDynamics::Math::VectorNd q, qdot, qddot, tau;

    // constructor
    RBDLModel() : is_initialized (false), ndof (-1), nadof (-1) {}

    // destructor
    ~RBDLModel() {}

    // update RBDL vectors
    void update_state (
      const double *sd, const double *u=NULL, const double *p=NULL
    ){
      for (int idof = 0; idof < ndof; ++idof) {
        q   [idof] = sd[idof];
        qdot[idof] = sd[ndof + idof];
        tau [idof] = 0.0;
      }

      if (u) {
        tau[0] = u[0];
      }
      if (p) {
        tau[1] = -p[0]*sd[3]; // tau[1] = -mu * pend_rte
      }
    }

  bool load_from_file (const char* filename, bool verbose=false) {
    if (!is_initialized) {
      if (!RigidBodyDynamics::Addons::LuaModelReadFromFile (
        filename, &model, verbose)
      ) {
        std::cerr << "Error loading LuaModel: " << filename << std::endl;
        abort();
      }

      // toggle state
      is_initialized = true;

      // get dimensions
      assert (model.dof_count == 2);
      ndof = model.dof_count;
      nadof = 1;

      // initialize states
      q     = RigidBodyDynamics::Math::VectorNd::Zero(ndof);
      qdot  = RigidBodyDynamics::Math::VectorNd::Zero(ndof);
      qddot = RigidBodyDynamics::Math::VectorNd::Zero(ndof);
      tau   = RigidBodyDynamics::Math::VectorNd::Zero(ndof);

      return true;
    } else {
      return false;
    }

  }

}; // END RBDLModel

// *****************************************************************************
// } // END NAMESPACE CommonCode
// *****************************************************************************
// #endif RBDL_MODEL_H_
#endif
// *****************************************************************************

