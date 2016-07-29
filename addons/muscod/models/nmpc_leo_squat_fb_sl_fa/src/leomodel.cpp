// -----------------------------------------------------------------------------
// RBDL Model Wrapper for convenient use in MUSCOD
//
// Copyright (c) 2016, Optimization in Robotics and Biomechanics
// (ORB), Heidelberg University, Germany
// All rights reserved
//
// Author(s): Manuel Kudruss <manuel.kudruss@iwr.uni-heidelberg.de>
// -----------------------------------------------------------------------------

#include <leomodel.h>
#include <../../../../../rbdl/include/grl/environments/LuaTypes.h>

// -----------------------------------------------------------------------------

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

// -----------------------------------------------------------------------------

LeoModel::LeoModel() :
	activeConstraintSet (""),
	nDof (-1),
	nActuatedDof(0),
	// nParameters(0)
	dynamicsComputed (false),
	impulsesComputed (false),
	kinematicsUpdated (false),
	momentumComputed (false)
{
}

// -----------------------------------------------------------------------------

double LeoModel::torque_from_voltage_and_angular_velocity (
	const double V, const double w
) {
	return Kt*G*(V - Kt*G*w)/R;
}

double LeoModel::voltage_from_torque_and_angular_velocity (
	const double tau, const double w
) {
	return (R*tau)/(Kt*G) + Kt*G*w;
}

// -----------------------------------------------------------------------------

void LeoModel::updateState (
	const double *sd, const double *u, const double *p_in,
	const string cs_name
) {
	activeConstraintSet = cs_name;

	dynamicsComputed = false;
	impulsesComputed = false;
	kinematicsUpdated = false;
	momentumComputed = false;

	for (unsigned int i = 0; i < nDof; i++) {
		q[i] = sd[i];
		qdot[i] = sd[i + nDof];
	}

	assert (nDof == nActuatedDof);
	for (unsigned int i = 0; i < nActuatedDof; i++) {
    //tau[i] = u[i] - 0.1*qdot[i];
		// TODO control on voltage level
    tau[i] = torque_from_voltage_and_angular_velocity (u[i], qdot[i]);
	}
}

// -----------------------------------------------------------------------------
// Update robot kinematics
void LeoModel::updateKinematics ()
{
  UpdateKinematics (model, q, qdot, qddot);

  kinematicsUpdated = true;
}

void LeoModel::updateForwardDynamics ()
{
	if (!activeConstraintSet.empty()) {
		// ForwardDynamicsContactsRangeSpaceSparse (
		ForwardDynamicsContactsNullSpace (
		// ForwardDynamicsContactsDirect (
			model, q, qdot, tau,
			constraints[activeConstraintSet],
			qddot
		);
	} else {
		ForwardDynamics (model, q, qdot, tau, qddot);
	}

	dynamicsComputed = true;
}

void LeoModel::updateInverseDynamics()
{
	// FIXME not sure what to use here
	InverseDynamics(model, q, qdot, qddot, tau);
	// inverseDynamicsComputed = true;
}

// -----------------------------------------------------------------------------
void LeoModel::calcForwardDynamicsRhs (double *res)
{
	// std::cout << "In " << __func__ << std::endl;
	// std::cout << "activeConstraintSet = " << activeConstraintSet << std::endl;
	if (!activeConstraintSet.empty()) {
		// std::cout << "evaluating 'ForwardDynamicsContactsNullSpace'" << std::endl;
		// ForwardDynamicsContactsRangeSpaceSparse (
		ForwardDynamicsContactsNullSpace (
		// ForwardDynamicsContactsDirect (
			model, q, qdot, tau,
			constraints[activeConstraintSet],
			qddot
		);
	} else {
		// std::cout << "evaluating 'ForwardDynamics'" << std::endl;
    ForwardDynamics (model, q, qdot, tau, qddot);
	}
	dynamicsComputed = true;

	for (unsigned int i = 0; i < nDof; i++) {
		res[i] = qdot[i];
		res[i + nDof] = qddot[i];
	}
}

void LeoModel::calcCollisionImpactRhs (double *res)
{
	// FIXME not sure what to use here
	// ComputeContactImpulsesRangeSpaceSparse (
	ComputeContactImpulsesDirect (
		model, q, qdot, constraints[activeConstraintSet], qdot_plus
	);

	for (unsigned int i = 0; i < nDof; i++) {
		res[i] = q[i];
		res[i + nDof] = qdot_plus[i];
	}

	impulsesComputed = true;
}

// -----------------------------------------------------------------------------
// Compute the position of a contact point wrt the base frame
Vector3d LeoModel::getPointPosition (string point_name)
{
	if (!kinematicsUpdated) {
		updateKinematics();
	}

	if ( !points.count( point_name ) ) {
		std::cerr << "ERROR in " << __func__ << std::endl;
		std::cerr << "Could not find point '" << point_name << "'!" << std::endl;
		std::cerr << "bailing out ..." << std::endl;
		abort();
	}

	Point point = points[point_name];
	unsigned int body_id = point.body_id;
	Vector3d point_local = point.point_local;

	return CalcBodyToBaseCoordinates (model, q, body_id, point_local, false);
}

// Compute the velocity of a point
Vector3d LeoModel::getPointVelocity (string point_name)
{
	if (!kinematicsUpdated) {
		updateKinematics();
	}

	if ( !points.count( point_name ) ) {
		std::cerr << "ERROR in " << __func__ << std::endl;
		std::cerr << "Could not find point '" << point_name << "'!" << std::endl;
		std::cerr << "bailing out ..." << std::endl;
		abort();
	}

	Point point = points[point_name];
	unsigned int body_id = point.body_id;
	Vector3d point_local = point.point_local;

	return CalcPointVelocity (model, q, qdot, body_id, point_local, false);
}

// Compute the force of a certain point according to the active constraint set
Vector3d LeoModel::getPointForce (string point_name)
{
	// check existence of points
	if ( !points.count( point_name ) ) {
		std::cerr << "ERROR in " << __func__ << std::endl;
		std::cerr << "Could not find point '" << point_name << "'!" << std::endl;
		std::cerr << "bailing out ..." << std::endl;
		abort();
	}

	// check existence of constraint set
	if ( !constraints.count( activeConstraintSet ) ) {
		std::cerr << "ERROR in " << __func__ << std::endl;
		std::cerr << "Could not find constraint set '" << activeConstraintSet << "'!" << std::endl;
		std::cerr << "bailing out ..." << std::endl;
		abort();
	}

	// calculate forward dynamics
	if (!dynamicsComputed) {
		// ForwardDynamicsContactsRangeSpaceSparse (
		ForwardDynamicsContactsNullSpace (
		// ForwardDynamicsContactsDirect (
			model, q, qdot, tau,
			constraints[activeConstraintSet],
			qddot
		);
		dynamicsComputed = true;
	}

	Vector3d result (0., 0., 0.);
	bool found = false;

	const RigidBodyDynamics::ConstraintSet &active_constraint_set = constraints[activeConstraintSet];
	const ConstraintSetInfo constraint_set_info = constraintSetInfos[activeConstraintSet];

	std::vector<ConstraintInfo>::const_iterator constraint_iter = constraintSetInfos[activeConstraintSet].constraints.begin();

	for (unsigned int ci = 0; ci < constraint_set_info.constraints.size(); ci++) {
		const ConstraintInfo& constraint_info = constraint_set_info.constraints[ci];

		if (constraint_info.point_name != point_name) {
			continue;
		}

		found = true;
		assert (constraint_info.normal == active_constraint_set.normal[ci]);

		result += active_constraint_set.force[ci] * active_constraint_set.normal[ci];
	}

	if (!found) {
		cerr << "Error (" << __func__ << "): Point '" << point_name;
		cerr << "' is not constrained in constraint set '";
		cerr << constraintSetInfos[activeConstraintSet].name << "'!" << endl;
		abort();
	}

	return result;
}

// -----------------------------------------------------------------------------

RigidBodyDynamics::Math::MatrixNd LeoModel::calcMassMatrix() {
	if (!kinematicsUpdated) {
		updateKinematics();
	}

	MatrixNd H (nDof, nDof);
	CompositeRigidBodyAlgorithm (model, q, H, false);
	return H;
}

// -----------------------------------------------------------------------------

RigidBodyDynamics::Math::MatrixNd LeoModel::calcContactJacobian() {
	if (!kinematicsUpdated) {
		updateKinematics();
	}

	unsigned int rows = constraintSetInfos[activeConstraintSet].constraints.size();
	MatrixNd G (rows, nDof);
	CalcContactJacobian(model, q, constraints[activeConstraintSet], G, true);
	return G;
}

// -----------------------------------------------------------------------------

void LeoModel::calcMomentum() {
	if (!kinematicsUpdated) {
		updateKinematics();
	}

	Utils::CalcCenterOfMass (
		model, q, qdot, modelMass, centerOfMass,
		&centerOfMassVelocity, &angularMomentum,
		false
	);

	momentumComputed = true;
}

Vector3d LeoModel::calcAngularMomentum() {
	if (!kinematicsUpdated) {
		updateKinematics();
	}

	if (!momentumComputed) {
		calcMomentum();
	}

	return angularMomentum;
}

double LeoModel::calcModelMass() {
	if (!kinematicsUpdated) {
		updateKinematics();
	}

	if (!momentumComputed) {
		calcMomentum();
	}

	return modelMass;
}

Vector3d LeoModel::calcCenterOfMass() {
	if (!kinematicsUpdated) {
		updateKinematics();
	}

	if (!momentumComputed) {
		calcMomentum();
	}

	return centerOfMass;
}

Vector3d LeoModel::calcCenterOfMassVelocity() {
	if (!kinematicsUpdated) {
		updateKinematics();
	}

	if (!momentumComputed) {
		calcMomentum();
	}

	return centerOfMassVelocity;
}

// -----------------------------------------------------------------------------

bool LeoModel::loadModelFromFile (const char* filename, bool verbose)
{
	if (!RigidBodyDynamics::Addons::LuaModelReadFromFile (filename, &model, verbose)) {
		cerr << "Error loading LuaModel: " << filename << endl;
		abort();
	}

  if (verbose)
  {
    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
      using namespace RigidBodyDynamics;
      using namespace RigidBodyDynamics::Math;
      Body &body = model.mBodies[i];
      SpatialRigidBodyInertia body_rbi = SpatialRigidBodyInertia::createFromMassComInertiaC(body.mMass, body.mCenterOfMass, body.mInertia);
      std::cout << "=============== Spatial inertia of body " << i << " ===============" << std::endl;
      std::cout <<  body_rbi.toMatrix() << std::endl;
    }
  }

	nDof = model.dof_count;
	nActuatedDof = nDof;

	assert (nActuatedDof >= 1 && nActuatedDof <= nDof);

	q = VectorNd::Zero (nDof);
	qdot = VectorNd::Zero (nDof);
	qdot_plus = VectorNd::Zero (nDof);
	qddot = VectorNd::Zero (nDof);
	// tau has size nDof but not every entry is nonzero!
	tau = VectorNd::Zero (nDof);

	return true;
}

// -----------------------------------------------------------------------------

bool LeoModel::loadPointsFromFile (const char* filename, bool verbose) {
	LuaTable lua_table = LuaTable::fromFile (filename);

	int point_count = lua_table["points"].length();

	for (int pi = 1; pi <= point_count; pi++) {
		Point point = lua_table["points"][pi];
		point.body_id = model.GetBodyId (point.body_name.c_str());

		points[point.name] = point;

		if (verbose) {
			cout << " Adding Point: " << point.name << endl;
			cout << "    body =        " << point.body_name << " (id = " << point.body_id << ")" << endl;
			cout << "    point_local = [" << point.point_local.transpose() << "]" << endl;
		}
	}

	// check whether we missed some points
	if (points.size() != point_count) {
		cerr << "Error: not all contact points have been loaded!" << endl;
		abort();
	}

	return true;
}

// -----------------------------------------------------------------------------

bool LeoModel::loadConstraintSetsFromFile (const char* filename, bool verbose) {
	// initialize Constraint Sets
	LuaTable lua_table = LuaTable::fromFile (filename);

	vector<LuaKey> constraint_set_keys = lua_table["constraint_sets"].keys();
	vector<string> constraint_set_names;
	for (size_t i = 0; i < constraint_set_keys.size(); i++) {
		if (constraint_set_keys[i].type == LuaKey::String) {
			constraint_set_names.push_back (constraint_set_keys[i].string_value);
		}
		else {
			cerr << "Found invalid constraint set name, string expected!" <<  endl;
			abort();
		}
	}

	for (size_t si = 0; si < constraint_set_names.size(); si++) {
		string set_name_str = constraint_set_names[si];

		if (verbose) {
			cout << "ConstraintSet '" << set_name_str << endl;
		}

		unsigned int constraint_count =
			lua_table["constraint_sets"][constraint_set_names[si].c_str()].length();

		ConstraintSet cs;
		ConstraintSetInfo csi;
		csi.name = set_name_str;
		csi.constraints.resize (constraint_count);

		for (int ci = 0; ci < constraint_count; ci++) {
			ConstraintInfo constraint_info =
			lua_table["constraint_sets"][set_name_str.c_str()][ci + 1];
			string point_name = constraint_info.point_name.c_str();
			constraint_info.point_id = ci;

			if (verbose) {
				cout << "  Adding Constraint point: " << points[point_name].name << endl;
				cout << "    body id = " << points[point_name].body_id << endl;
				cout << "    normal =  [" << constraint_info.normal.transpose() << "]" << endl;
			}
			cs.AddConstraint (
				points[point_name].body_id,
				points[point_name].point_local,
				constraint_info.normal
			);
			csi.constraints[ci] = constraint_info;
		}

		// save constraint set infos
		constraintSetInfos[set_name_str] = csi;

		// assign constraint set
		constraints[set_name_str] = cs;
		// TODO check which solver works better
		constraints[set_name_str].linear_solver = LinearSolverHouseholderQR;
		// constraintSets[set_name].linear_solver = LinearSolverPartialPivLU;
		constraints[set_name_str].Bind (model);
	}

	// check whether we missed some sets
	if (constraints.size() != constraint_set_names.size()) {
		cerr << "Error: not all constraint sets have been loaded!" << endl;
		abort();
	}

	return true;
}

// -----------------------------------------------------------------------------

// string model_path = "leo.lua";
// string model_path = "leo_dl.lua";
string model_path = "leo_fb_sl.lua";

int main(int argc, char const *argv[])
{
	// Small test case for model wrapper
	cout << "Leo Model Test Case" << endl;

	// Initialize model from lua file
	cout << "Initializing model ..." << endl;
	LeoModel leo;
	leo.loadModelFromFile (model_path.c_str());
	cout << "... successful!" << endl << endl;

	// Load contact points
	cout << "Loading points ..." << endl;
	leo.loadPointsFromFile (model_path.c_str(), true);
	cout << "... successful!" << endl << endl;

	// Load constraint sets
	cout << "Loading constraint sets ..." << endl;
	leo.loadConstraintSetsFromFile (model_path.c_str(), true);
	cout << "... successful!" << endl;

	VectorNd q = VectorNd::Zero(leo.nDof);
	VectorNd qdot = VectorNd::Zero(leo.nDof);
	VectorNd qddot = VectorNd::Zero(leo.nDof);
	VectorNd tau = VectorNd::Zero(leo.nDof);
	VectorNd rhs = VectorNd::Zero(2*leo.nDof);
	// provide initial values
	q <<
	 0.05,
	-0.1,
	 0.1,
	 0.0;
	//  0.1,
	// -0.1,
	//  0.05;

	// assign them to model
	leo.q = q;
	leo.qdot = qdot;
	leo.qddot = qddot;
	leo.activeConstraintSet = "";

	cout << "Computing inverse dynamics ... " << endl;
	cout << " q:      " << leo.q.transpose() << endl;
	cout << " qdot:   " << leo.qdot.transpose() << endl;
	cout << "for desired accelerations:" << endl;
	cout << " qddot:  " << leo.qddot.transpose() << endl;
	leo.updateInverseDynamics();
	cout << "needed torques:" << endl;
	cout << " taus:   " << leo.tau.transpose() << endl;
	cout << "... successful!" << endl;

	cout << "Checking forward dynamics ... " << endl;
	cout << " q:      " << leo.q.transpose() << endl;
	cout << " qdot:   " << leo.qdot.transpose() << endl;
	cout << "for given torques:" << endl;
	cout << " taus:   " << leo.tau.transpose() << endl;
	leo.updateForwardDynamics();
	cout << "resulting accelerations:" << endl;
	cout << " qddot:  " << leo.qddot.transpose() << endl;
	leo.calcForwardDynamicsRhs(rhs.data());
	cout << "resulting accelerations:" << endl;
	cout << " rhs:  " << rhs.transpose() << endl;
	cout << "... successful!" << endl;
	cout << endl;

	// provide initial values
	leo.tau <<
	 0.1,
	 0.1,
	 0.1,
	 0.1;
	//  0.1,
	// -0.1,
	//  0.05;
	cout << "Checking forward dynamics ... " << endl;
	cout << " q:      " << leo.q.transpose() << endl;
	cout << " qdot:   " << leo.qdot.transpose() << endl;
	cout << "for given torques:" << endl;
	cout << " taus:   " << leo.tau.transpose() << endl;
	leo.updateForwardDynamics();
	cout << "resulting accelerations:" << endl;
	cout << " qddot:  " << leo.qddot.transpose() << endl;
	cout << endl;
	leo.calcForwardDynamicsRhs(rhs.data());
	cout << "resulting accelerations:" << endl;
	cout << " rhs:  " << rhs.transpose() << endl;
	cout << "... successful!" << endl;
	cout << endl;

	return 0;
	// // calculate mass matrix
	// MatrixNd H = leo.calcMassMatrix();
	// cout << "Computing mass matrix ... " << endl;
	// cout << H << endl;
	// cout << "... successful!" << endl;

	// // calculate constraint Jacobian for given constraint set
	// MatrixNd G = leo.calcContactJacobian();
	// cout << "Computing contact Jacobian ... " << endl;
	// cout << G << endl;
	// cout << G.transpose() << endl;
	// cout << "... successful!" << endl;


	// return 0;
}

// -----------------------------------------------------------------------------
