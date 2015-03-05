#
# CMake build file for configuration library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 29-03-2010 (wcaarls): Initial revision
#

# Preamble
PROJECT(dbl_configuration)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Setup project environment
GET_FILENAME_COMPONENT(BASE_DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(WORKSPACE_DIR ${BASE_DIR}/../../../..)
SET(TARGET dbl_configuration)

# Specify sources
ADD_LIBRARY(${TARGET} STATIC ${BASE_DIR}/Configuration.cpp
                      ${BASE_DIR}/XMLConfiguration.cpp
           )

INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/logging.cmake)
