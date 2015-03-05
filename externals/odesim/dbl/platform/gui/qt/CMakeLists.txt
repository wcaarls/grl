#
# CMake build file for qt library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 29-03-2010 (wcaarls): Initial revision
#

# Preamble
PROJECT(dbl_qt)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Setup project environment
GET_FILENAME_COMPONENT(BASE_DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(WORKSPACE_DIR ${BASE_DIR}/../../../..)
SET(TARGET dbl_qt)

# Qt stuff
SET(QT_USE_QTOPENGL TRUE)
find_package(Qt4 REQUIRED)

IF(QT4_FOUND)  
  include(${QT_USE_FILE})
  INCLUDE_DIRECTORIES(
  ${QT_INCLUDE_DIR}      
  )
ENDIF(QT4_FOUND)

SET(qt_MOC_HDRS ${BASE_DIR}/GLWidget.h)

QT4_WRAP_CPP(qt_MOC_SRCS ${qt_MOC_HDRS})

INCLUDE (${WORKSPACE_DIR}/dbl/platform/include.cmake)

# Specify sources
ADD_LIBRARY(${TARGET} STATIC ${BASE_DIR}/GLWidget.cpp
                      ${qt_MOC_SRCS}
           )
