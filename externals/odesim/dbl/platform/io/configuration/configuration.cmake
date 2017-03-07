#
# CMake include file for configuration library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 29-03-2010 (wcaarls): Initial revision
#

INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/logging.cmake)

INCLUDE_DIRECTORIES(${WORKSPACE_DIR}/dbl/platform/io/configuration)
TARGET_LINK_LIBRARIES(${TARGET} dbl_configuration)

IF (NOT __CONFIGURATION_CMAKE_INCLUDED)
  SET(__CONFIGURATION_CMAKE_INCLUDED 1)

  ADD_SUBDIRECTORY(${WORKSPACE_DIR}/dbl/platform/io/configuration ${CMAKE_BINARY_DIR}/externals/dbl/platform/io/configuration)
ENDIF (NOT __CONFIGURATION_CMAKE_INCLUDED)
