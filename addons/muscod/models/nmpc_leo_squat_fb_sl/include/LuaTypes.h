/*
 * Copyright (c) 2011-2016, Optimization in Robotics and Biomechanics
 * (ORB), Heidelberg University, Germany
 * All rights reserved
 *
 * Author(s): Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Directed by Prof. Katja Mombaur <katja.mombaur@iwr.uni-heidelberg.de>
 */

#ifndef MARKER_MODEL_LUA_TYPES
#define MARKER_MODEL_LUA_TYPES

#include <rbdl/rbdl_math.h>
#include <rbdl/addons/luamodel/luatables.h>

#include "LuaBasic.h"

template<> inline RigidBodyDynamics::Math::Vector3d LuaTableNode::getDefault<RigidBodyDynamics::Math::Vector3d>(const RigidBodyDynamics::Math::Vector3d &default_value) {
	RigidBodyDynamics::Math::Vector3d result = default_value;

	if (stackQueryValue()) {
		LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

		if (vector_table.length() != 3) {
			std::cerr << "LuaModel Error: invalid 3d vector!" << std::endl;
			abort();
		}

		result[0] = vector_table[1];
		result[1] = vector_table[2];
		result[2] = vector_table[3];
	}

	stackRestore();

	return result;
}

template<> inline RigidBodyDynamics::Math::SpatialVector LuaTableNode::getDefault<RigidBodyDynamics::Math::SpatialVector>(const RigidBodyDynamics::Math::SpatialVector &default_value) {
	RigidBodyDynamics::Math::SpatialVector result = default_value;

	if (stackQueryValue()) {
		LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

		if (vector_table.length() != 6) {
			std::cerr << "LuaModel Error: invalid 6d vector!" << std::endl;
			abort();
		}
		result[0] = vector_table[1];
		result[1] = vector_table[2];
		result[2] = vector_table[3];
		result[3] = vector_table[4];
		result[4] = vector_table[5];
		result[5] = vector_table[6];
	}

	stackRestore();

	return result;
}

template<> inline RigidBodyDynamics::Math::Matrix3d LuaTableNode::getDefault<RigidBodyDynamics::Math::Matrix3d>(const RigidBodyDynamics::Math::Matrix3d &default_value) {
	RigidBodyDynamics::Math::Matrix3d result = default_value;

	if (stackQueryValue()) {
		LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

		if (vector_table.length() != 3) {
			std::cerr << "LuaModel Error: invalid 3d matrix!" << std::endl;
			abort();
		}

		if (vector_table[1].length() != 3
				|| vector_table[2].length() != 3
				|| vector_table[3].length() != 3) {
			std::cerr << "LuaModel Error: invalid 3d matrix!" << std::endl;
			abort();
		}

		result(0,0) = vector_table[1][1];
		result(0,1) = vector_table[1][2];
		result(0,2) = vector_table[1][3];

		result(1,0) = vector_table[2][1];
		result(1,1) = vector_table[2][2];
		result(1,2) = vector_table[2][3];

		result(2,0) = vector_table[3][1];
		result(2,1) = vector_table[3][2];
		result(2,2) = vector_table[3][3];
	}

	stackRestore();

	return result;
}

template<> inline RigidBodyDynamics::Math::SpatialTransform LuaTableNode::getDefault<RigidBodyDynamics::Math::SpatialTransform>(const RigidBodyDynamics::Math::SpatialTransform &default_value) {
	RigidBodyDynamics::Math::SpatialTransform result = default_value;

	if (stackQueryValue()) {
		LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

		result.r = vector_table["r"].getDefault<RigidBodyDynamics::Math::Vector3d>(RigidBodyDynamics::Math::Vector3d::Zero(3));
		result.E = vector_table["E"].getDefault<RigidBodyDynamics::Math::Matrix3d>(RigidBodyDynamics::Math::Matrix3d::Identity (3,3));
	}

	stackRestore();

	return result;
}

template<> inline Point LuaTableNode::getDefault<Point>(const Point &default_value) {
	Point result = default_value;

	if (stackQueryValue()) {
		LuaTable point_table = LuaTable::fromLuaState (luaTable->L);

		result.name = point_table["name"].get<std::string>();
		result.point_local = point_table["point"];
		result.body_name = point_table["body"].get<std::string>();
	}

	stackRestore();

	return result;
}

template<> inline ConstraintInfo LuaTableNode::getDefault<ConstraintInfo>(const ConstraintInfo &default_value) {
	ConstraintInfo result = default_value;

	if (stackQueryValue()) {
		LuaTable constraint_table = LuaTable::fromLuaState (luaTable->L);

		result.point_name = constraint_table["point"].get<std::string>();
		result.normal = constraint_table["normal"];
	}

	stackRestore();

	return result;
}

/* MARKER_MODEL_LUA_TYPES */
#endif
