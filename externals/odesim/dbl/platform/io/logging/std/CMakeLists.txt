#
# CMake build file for standard logging library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 15-04-2010 (wcaarls): Initial revision
#

# Preamble
PROJECT(dbl_stdlogging)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Setup project environment
GET_FILENAME_COMPONENT(BASE_DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(WORKSPACE_DIR ${BASE_DIR}/../../../../..)
SET(TARGET dbl_stdlogging)

# Specify sources
ADD_LIBRARY(${TARGET} STATIC ${BASE_DIR}/StdLog2.cpp
           )

INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/logging.cmake)
