#
# CMake include file for standard logging library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 15-04-2010 (wcaarls): Initial revision
#

INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/logging.cmake)

TARGET_LINK_LIBRARIES(${TARGET} dbl_stdlogging)

IF (NOT __STDLOGGING_CMAKE_INCLUDED)
  SET(__STDLOGGING_CMAKE_INCLUDED 1)

  ADD_SUBDIRECTORY(${WORKSPACE_DIR}/dbl/platform/io/logging/std ${CMAKE_BINARY_DIR}/externals/dbl/platform/io/logging/std)
ENDIF (NOT __STDLOGGING_CMAKE_INCLUDED)
