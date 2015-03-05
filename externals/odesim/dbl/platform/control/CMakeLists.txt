#
# CMake build file for control library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 16-04-2010 (wcaarls): Initial revision
#

# Preamble
PROJECT(dbl_control)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Setup project environment
GET_FILENAME_COMPONENT(BASE_DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(WORKSPACE_DIR ${BASE_DIR}/../../..)
SET(TARGET dbl_control)

INCLUDE (${WORKSPACE_DIR}/dbl/platform/include.cmake)

# Specify sources
ADD_LIBRARY(${TARGET} STATIC ${BASE_DIR}/GenericState.cpp
           )

INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/configuration/configuration.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/logging.cmake)
