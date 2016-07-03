// -----------------------------------------------------------------------------
// Copyright (c) 2016, Optimization in Robotics and Biomechanics
// (ORB), Heidelberg University, Germany
// All rights reserved
//
// Author(s): Manuel Kudruss <manuel.kudruss@iwr.uni-heidelberg.de>
// -----------------------------------------------------------------------------
#ifndef _LEO_MODEL_H
#define _LEO_MODEL_H
// -----------------------------------------------------------------------------

#include <vector>
#include <string>
#include <limits>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/luamodel/luamodel.h>
#include <rbdl/addons/luamodel/luatables.h>

#include <../../../../../rbdl/include/grl/environments/LuaBasic.h>

// -----------------------------------------------------------------------------
struct LeoModel {
    LeoModel();

    // Currently selected constraint set, specified by LeoModel::updateState()
    std::string activeConstraintSet;
    // Number of actual degrees of freedom
    unsigned int nDof;
    // Number of actuated degrees of freedom. tau[0] .. tau[nDof - nActuatedDof] is always 0 (unactuated)
    unsigned int nActuatedDof;
    // Number of parameters, i.e. value of ParamIdLast
    // unsigned int nParameters;

    // Flags that minimize kinematics and dynamics computations to a minimum
    bool dynamicsComputed;
    bool impulsesComputed;
    bool kinematicsUpdated;
    bool momentumComputed;

    // The actual RBDL model
    RigidBodyDynamics::Model model;

    // Values that are used when calling RBDL
    // RigidBodyDynamics::Math::VectorNd p;
    RigidBodyDynamics::Math::VectorNd q;
    RigidBodyDynamics::Math::VectorNd qdot;
    RigidBodyDynamics::Math::VectorNd qdot_plus;
    RigidBodyDynamics::Math::VectorNd qddot;
    RigidBodyDynamics::Math::VectorNd tau;

    // Centroidal quantities
    double modelMass;
    RigidBodyDynamics::Math::Vector3d angularMomentum;
    RigidBodyDynamics::Math::Vector3d centerOfMass;
    RigidBodyDynamics::Math::Vector3d centerOfMassVelocity;

    // Model properties
    std::map<std::string, Point> points;

    // Information of the constraint sets (mostly used when parsing Lua file)
    // NOTE: constraintSetInfos is needed because point information is stored
    std::map<std::string, ConstraintSetInfo> constraintSetInfos;
    std::map<std::string, RigidBodyDynamics::ConstraintSet> constraints;

    // Copies state information from MUSCOD to the model and switches to the given constraint set.
    void updateState (
        const double* sd, const double* u, const double* p_in,
        std::string cs_name
    );
    // Updates the kinematics of the RDBL model (is called automatically when required)
    void updateKinematics();
    // Update the dynamics by computing forward dynamics, this should be called after updateState in order to compute the correct dynamics
    void updateForwardDynamics();
    void updateInverseDynamics();

    // conversion of torques to/from voltages for given angular velocity of dynamixel
    const double Kt = 0.00992;
    const double G = 193.0;
    const double R = 8.6;
    double torque_from_voltage_and_angular_velocity (const double V, const double w);
    double voltage_from_torque_and_angular_velocity (const double tau, const double w);

    // Computes the forward dynamics for the model and active constraint set
    void calcForwardDynamicsRhs (double *res);
    // Computes the change in velocities due to a collision
    void calcCollisionImpactRhs (double *res);

    // Returns 3D vectors containing position, velocity or force of a point attached to a body, the point has to be one specified in the points list
    RigidBodyDynamics::Math::Vector3d getPointPosition (std::string point_name);
    RigidBodyDynamics::Math::Vector3d getPointVelocity (std::string point_name);
    RigidBodyDynamics::Math::Vector3d getPointForce (std::string point_name);

    // Returns mass matrix
    RigidBodyDynamics::Math::MatrixNd calcMassMatrix();

    // Returns contact Jacobian for the active constraint set
    RigidBodyDynamics::Math::MatrixNd calcContactJacobian();

    // Centroidal quantities
    double calcModelMass();
    void calcMomentum();
    RigidBodyDynamics::Math::Vector3d calcAngularMomentum();
    RigidBodyDynamics::Math::Vector3d calcCenterOfMass();
    RigidBodyDynamics::Math::Vector3d calcCenterOfMassVelocity();

    bool loadModelFromFile (const char* filename, bool verbose=false);
    bool loadPointsFromFile (const char* filename, bool verbose=false);
    bool loadConstraintSetsFromFile (const char* filename, bool verbose=false);
};

// -----------------------------------------------------------------------------
/* _LEO_MODEL_H */
#endif
// -----------------------------------------------------------------------------
