# Setup build environment
set(TARGET addon_leosim)

set(WORKSPACE_DIR ${SRC}/../../../externals/odesim)

message("-- Building leosim addon")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# Build library
add_library(${TARGET} SHARED
                      ${SRC}/leosim.cpp
                      ${SRC}/LeoBhWalkSym.cpp
                      ${SRC}/STGLeoSim.cpp
                      ${SRC}/ThirdOrderButterworth.cpp)

include_directories(${SRC}/../include/grl/environments/leosim)

INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/configuration/configuration.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/stdlogging.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/externals/bithacks/bithacks.cmake)

add_definitions(-DLEOSIM_CONFIG_DIR="${SRC}/../cfg")

# Add dependencies
grl_link_libraries(${TARGET} base addons/odesim)
target_link_libraries(${TARGET})

install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
