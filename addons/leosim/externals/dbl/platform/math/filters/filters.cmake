#
# CMake include file for filters library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 29-03-2010 (wcaarls): Initial revision
#

INCLUDE_DIRECTORIES(${WORKSPACE_DIR}/dbl/platform/math/filters)
TARGET_LINK_LIBRARIES(${TARGET} leosim_filters)

IF (NOT __FILTERS_CMAKE_INCLUDED)
  SET(__FILTERS_CMAKE_INCLUDED 1)

  ADD_SUBDIRECTORY(${WORKSPACE_DIR}/dbl/platform/math/filters ${WORKSPACE_DIR}/build/filters/${CMAKE_BUILD_TYPE}${COMPILER_VERSION})
ENDIF (NOT __FILTERS_CMAKE_INCLUDED)
