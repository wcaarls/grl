#ifndef LUABASIC_H
#define LUABASIC_H

/** Data of a single point */
struct Point {
    Point() :
        name ("unknown"),
        body_id (-1),
        body_name (""),
        point_local (
                std::numeric_limits<double>::signaling_NaN(),
                std::numeric_limits<double>::signaling_NaN(),
                std::numeric_limits<double>::signaling_NaN()
                )
    { }

    std::string name;
    unsigned int body_id;
    std::string body_name;
    RigidBodyDynamics::Math::Vector3d point_local;
};

// -----------------------------------------------------------------------------

/** Data of a single constraint */
struct ConstraintInfo {
    ConstraintInfo() :
        point_id (-1),
        point_name (""),
        normal (
                std::numeric_limits<double>::signaling_NaN(),
                std::numeric_limits<double>::signaling_NaN(),
                std::numeric_limits<double>::signaling_NaN()
                ) {
    }
    unsigned int point_id;
    std::string point_name;
    RigidBodyDynamics::Math::Vector3d normal;
};

/** Structure that holds data of a complete constraint set */
struct ConstraintSetInfo {
    ConstraintSetInfo() :
        name ("undefined") {
    }
    std::vector<ConstraintInfo> constraints;
    std::string name;
};

// -----------------------------------------------------------------------------

#endif // LUABASIC_H
