#
# CMake include file for odesim library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 29-03-2010 (wcaarls): Initial revision
#

INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/configuration/configuration.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/threading/threading.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/control/control.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/gui/qt/qt.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/simulation/genericsim.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/simulation/simvis/simvis.cmake)

INCLUDE_DIRECTORIES(${WORKSPACE_DIR}/dbl/platform/simulation/odesim)
TARGET_LINK_LIBRARIES(${TARGET} dbl_odesim)

IF (ODE_LIB)
  TARGET_LINK_LIBRARIES(${TARGET} ${ODE_LIB})
ELSE ()
  TARGET_LINK_LIBRARIES(${TARGET} ode)
ENDIF (ODE_LIB)

IF (NOT __ODESIM_CMAKE_INCLUDED)
  SET(__ODESIM_CMAKE_INCLUDED 1)

  ADD_SUBDIRECTORY(${WORKSPACE_DIR}/dbl/platform/simulation/odesim ${CMAKE_BINARY_DIR}/externals/dbl/platform/simulation/odesim)
ENDIF (NOT __ODESIM_CMAKE_INCLUDED)
