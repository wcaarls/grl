#
# CMake include file for math library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 29-03-2010 (wcaarls): Initial revision
#

INCLUDE_DIRECTORIES(${WORKSPACE_DIR}/dbl/platform/math)
TARGET_LINK_LIBRARIES(${TARGET} leosim_math)

IF (NOT __MATH_CMAKE_INCLUDED)
  SET(__MATH_CMAKE_INCLUDED 1)

  ADD_SUBDIRECTORY(${WORKSPACE_DIR}/dbl/platform/math ${WORKSPACE_DIR}/build/math/${CMAKE_BUILD_TYPE}${COMPILER_VERSION})
ENDIF (NOT __MATH_CMAKE_INCLUDED)
