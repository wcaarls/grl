# Setup build environment
set(TARGET addon_leosim)

set(WORKSPACE_DIR ${SRC}/../../../externals/odesim)

message("-- Building leosim addon")

ADD_DEFINITIONS(-DCONFIG_DIR="${SRC}/../cfg")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# Build library
add_library(${TARGET} SHARED
                      ${SRC}/leosim.cpp
                      ${SRC}/../externals/LEO/control/controllers/learning/LeoBhWalkSym.cpp
                      ${SRC}/../externals/LEO/leosim/STGLeoSim.cpp)

INCLUDE_DIRECTORIES(${SRC}/../externals/LEO/control/controllers/learning)
INCLUDE_DIRECTORIES(${SRC}/../externals/LEO/leosim)

INCLUDE (${SRC}/../externals/leo.cmake)

# Add dependencies
grl_link_libraries(${TARGET} base)
target_link_libraries(${TARGET})

message("Dest " ${CMAKE_CXX_FLAGS})

install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")

