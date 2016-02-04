#
# CMake include file for Leo's control library
# Ivan Koryakovskiy <i.koryakovskiy@gmail.com>
#
# 28-01-2016: Initial revision
#

# Include dbl files independently
set(TEMP_WORKSPACE_DIR ${WORKSPACE_DIR})
set(WORKSPACE_DIR ${SRC}/../externals)
  INCLUDE (${SRC}/../externals/dbl/platform/math/math.cmake)
  INCLUDE (${SRC}/../externals/dbl/platform/math/half/half.cmake)
  INCLUDE (${SRC}/../externals/dbl/platform/math/filters/filters.cmake)
  INCLUDE (${SRC}/../externals/dbl/platform/time/time.cmake)
  INCLUDE_DIRECTORIES(${SRC}/../externals/dbl/platform/learning)
  INCLUDE_DIRECTORIES(${SRC}/../externals/dbl/platform/control)
set(WORKSPACE_DIR ${TEMP_WORKSPACE_DIR})

# Include Leo files with support from odesim
INCLUDE (${SRC}/../externals/LEO/control/STG/leostg.cmake)
INCLUDE (${SRC}/../externals/LEO/control/controllers/leocontrollers.cmake)
INCLUDE_DIRECTORIES(${SRC}/../externals/LEO/control/STG)

IF (NOT __LEOCONTROL_CMAKE_INCLUDED)
  SET(__LEOCONTROL_CMAKE_INCLUDED 1)

ENDIF (NOT __LEOCONTROL_CMAKE_INCLUDED)
