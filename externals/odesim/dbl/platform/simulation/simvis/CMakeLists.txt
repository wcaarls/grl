#
# CMake build file for simvis library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 29-03-2010 (wcaarls): Initial revision
#

# Preamble
PROJECT(dbl_simvis)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Setup project environment
GET_FILENAME_COMPONENT(BASE_DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(WORKSPACE_DIR ${BASE_DIR}/../../../..)
SET(TARGET dbl_simvis)

INCLUDE (${WORKSPACE_DIR}/dbl/platform/include.cmake)

IF (WITH_GUI)
  SET(GUI_SRCS SimVisWidget.cpp)
ENDIF (WITH_GUI)

# Specify sources
ADD_LIBRARY(${TARGET} STATIC
			SimVis.cpp
			${GUI_SRCS}
           )

INCLUDE (${WORKSPACE_DIR}/dbl/platform/simulation/genericsim.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/configuration/configuration.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/gui/qt/qt.cmake)
