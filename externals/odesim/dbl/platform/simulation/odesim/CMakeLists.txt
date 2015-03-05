#
# CMake build file for odesim library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 29-03-2010 (wcaarls): Initial revision
#

# Preamble
PROJECT(dbl_odesim)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Setup project environment
GET_FILENAME_COMPONENT(BASE_DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(WORKSPACE_DIR ${BASE_DIR}/../../../..)
SET(TARGET dbl_odesim)

INCLUDE (${WORKSPACE_DIR}/dbl/platform/include.cmake)

# Specify sources
ADD_LIBRARY(${TARGET} STATIC ODEBodies.cpp
                      ODECollisions.cpp
                      ODEGeoms.cpp
                      ODEJointMotors.cpp
                      ODEJoints.cpp
                      ODEObjects.cpp
                      ODESim.cpp
                      GenericODESim.cpp
           )

INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/configuration/configuration.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/threading/threading.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/control/control.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/simulation/genericsim.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/simulation/simvis/simvis.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/gui/qt/qt.cmake)
