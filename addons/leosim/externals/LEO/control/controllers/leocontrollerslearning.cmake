#
# CMake include file for Leo's controllers library
# Erik Schuitema <e.schuitema@tudelft.nl>
#
# 30-03-2010: Initial revision
#

#INCLUDE(${WORKSPACE_DIR}/dbl/platform/io/logging/logging.cmake)
#INCLUDE(${WORKSPACE_DIR}/dbl/platform/io/configuration/configuration.cmake)
INCLUDE(${WORKSPACE_DIR}/LEO/control/STG/leostglearning.cmake)

INCLUDE_DIRECTORIES(${WORKSPACE_DIR}/LEO/control/controllers/learning)

IF (NOT __LEOCONTROLLERSLEARNING_CMAKE_INCLUDED)
  SET(__LEOCONTROLLERSLEARNING_CMAKE_INCLUDED 1)

ENDIF (NOT __LEOCONTROLLERSLEARNING_CMAKE_INCLUDED)
