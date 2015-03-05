#
# CMake include file for genericsim
# Erik Schuitema <e.schuitema@tudelft.nl>
#

INCLUDE_DIRECTORIES (${WORKSPACE_DIR}/dbl/platform/simulation)

 # Shame :( TODO: remove dependency on ODE, but no time at the moment..
IF (ODE_LIB)
  TARGET_LINK_LIBRARIES(${TARGET} ${ODE_LIB})
ELSE ()
  TARGET_LINK_LIBRARIES(${TARGET} ode)
ENDIF (ODE_LIB)

