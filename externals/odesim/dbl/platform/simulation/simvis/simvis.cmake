#
# CMake include file for simvis library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 29-03-2010 (wcaarls): Initial revision
#

INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/configuration/configuration.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/gui/qt/qt.cmake)

INCLUDE_DIRECTORIES (${WORKSPACE_DIR}/dbl/platform/simulation/simvis)
TARGET_LINK_LIBRARIES(${TARGET} dbl_simvis)

IF (NOT __SIMVIS_CMAKE_INCLUDED)
  SET(__SIMVIS_CMAKE_INCLUDED 1)

  ADD_SUBDIRECTORY(${WORKSPACE_DIR}/dbl/platform/simulation/simvis ${CMAKE_BINARY_DIR}/externals/dbl/platform/simulation/simvis)
ENDIF (NOT __SIMVIS_CMAKE_INCLUDED)
