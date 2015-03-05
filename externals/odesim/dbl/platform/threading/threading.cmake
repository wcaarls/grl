#
# CMake include file for threading library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 29-03-2010 (wcaarls): Initial revision
#

INCLUDE_DIRECTORIES(${WORKSPACE_DIR}/dbl/platform/threading)

IF (PTHREAD_LIB)
  TARGET_LINK_LIBRARIES(${TARGET} ${PTHREAD_LIB})
ELSE ()
  TARGET_LINK_LIBRARIES(${TARGET} pthread rt)
ENDIF (PTHREAD_LIB)

IF (NOT __THREADING_CMAKE_INCLUDED)
  SET(__THREADING_CMAKE_INCLUDED 1)

ENDIF (NOT __THREADING_CMAKE_INCLUDED)
