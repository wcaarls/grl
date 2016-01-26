# 
# CMake include file of Leo's STG headers only - no leostg will be built
# Erik Schuitema 2010 (e.schuitema@tudelft.nl)
# 

# Include dependencies
INCLUDE(${WORKSPACE_DIR}/dbl/platform/control/control.cmake)
INCLUDE(${WORKSPACE_DIR}/dbl/externals/bithacks/bithacks.cmake)

# Include directories
INCLUDE_DIRECTORIES(${WORKSPACE_DIR}/LEO/control/STG)

