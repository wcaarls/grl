#
# CMake build file for half library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 29-03-2010 (wcaarls): Initial revision
#

# Preamble
PROJECT(leosim_half)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Setup project environment
GET_FILENAME_COMPONENT(BASE_DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(WORKSPACE_DIR ${BASE_DIR}/../../../..)
SET(TARGET leosim_half)

# Specify sources
ADD_LIBRARY(${TARGET} STATIC ${BASE_DIR}/half.cpp)
