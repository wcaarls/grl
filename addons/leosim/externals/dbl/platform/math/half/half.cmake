#
# CMake include file for half library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 29-03-2010 (wcaarls): Initial revision
#

INCLUDE_DIRECTORIES(${WORKSPACE_DIR}/dbl/platform/math/half)
TARGET_LINK_LIBRARIES(${TARGET} leosim_half)

IF (NOT __HALF_CMAKE_INCLUDED)
  SET(__HALF_CMAKE_INCLUDED 1)

  ADD_SUBDIRECTORY(${WORKSPACE_DIR}/dbl/platform/math/half ${WORKSPACE_DIR}/build/half/${CMAKE_BUILD_TYPE}${COMPILER_VERSION})
ENDIF (NOT __HALF_CMAKE_INCLUDED)
