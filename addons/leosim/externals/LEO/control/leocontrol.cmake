#
# CMake include file for Leo's control library
# Erik Schuitema <e.schuitema@tudelft.nl>
#
# 30-03-2010: Initial revision
#

INCLUDE (${WORKSPACE_DIR}/LEO/control/STG/leostg.cmake)
INCLUDE (${WORKSPACE_DIR}/LEO/control/controllers/leocontrollers.cmake)

#No headers in this dir yet: INCLUDE_DIRECTORIES(${WORKSPACE_DIR}/LEO/control)

IF (NOT __LEOCONTROL_CMAKE_INCLUDED)
  SET(__LEOCONTROL_CMAKE_INCLUDED 1)

ENDIF (NOT __LEOCONTROL_CMAKE_INCLUDED)
