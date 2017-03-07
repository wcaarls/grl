#
# CMake include file for control library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 29-03-2010 (wcaarls): Initial revision
# 16-04-2010 (wcaarls): Create library
#

INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/configuration/configuration.cmake)
#INCLUDE (${WORKSPACE_DIR}/dbl/externals/muparser/muparser.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/logging.cmake)

INCLUDE_DIRECTORIES(${WORKSPACE_DIR}/dbl/platform/control)
TARGET_LINK_LIBRARIES(${TARGET} dbl_control)

IF (NOT __CONTROL_CMAKE_INCLUDED)
  SET(__CONTROL_CMAKE_INCLUDED 1)

  ADD_SUBDIRECTORY(${WORKSPACE_DIR}/dbl/platform/control ${CMAKE_BINARY_DIR}/externals/dbl/platform/control)
ENDIF (NOT __CONTROL_CMAKE_INCLUDED)
