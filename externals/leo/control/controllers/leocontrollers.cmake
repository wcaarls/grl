#
# CMake include file for Leo's controllers library
# Erik Schuitema <e.schuitema@tudelft.nl>
#
# 30-03-2010: Initial revision
#

INCLUDE(${WORKSPACE_DIR}/dbl/platform/io/logging/logging.cmake)
INCLUDE(${WORKSPACE_DIR}/dbl/platform/io/configuration/configuration.cmake)

INCLUDE_DIRECTORIES(${WORKSPACE_DIR}/LEO/control/controllers/calibration)
INCLUDE_DIRECTORIES(${WORKSPACE_DIR}/LEO/control/controllers/common)
INCLUDE_DIRECTORIES(${WORKSPACE_DIR}/LEO/control/controllers/standup)
INCLUDE_DIRECTORIES(${WORKSPACE_DIR}/LEO/control/controllers/walking)

IF (NOT __LEOCONTROLLERS_CMAKE_INCLUDED)
  SET(__LEOCONTROLLERS_CMAKE_INCLUDED 1)

ENDIF (NOT __LEOCONTROLLERS_CMAKE_INCLUDED)
