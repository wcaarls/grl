project(grl)
cmake_minimum_required(VERSION 2.8.3)
include(grl.cmake)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
  message("-- Set default build type: ${CMAKE_BUILD_TYPE}")
else()
  message("-- Build type: ${CMAKE_BUILD_TYPE}")
endif()

# Find Eigen
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN3 REQUIRED eigen3)

# Use OpenMP if available
find_package(OpenMP)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++0x -Wall -Wextra -Wno-sign-compare -Wno-unused-parameter -Wno-variadic-macros -Wno-pedantic ${OpenMP_CXX_FLAGS}")

# List of modules to build
set(MODULES base
            externals/yaml-cpp
            externals/ann
            externals/cma
            externals/ics
            externals/itc
            externals/rbdl
            externals/cppzmq
            addons/cma
            addons/llr
            addons/gl
            addons/glut
            addons/ros
            addons/matlab
            addons/odesim
            addons/rbdl
            addons/muscod
            addons/leo
            addons/leo2
            addons/tlm
            addons/lqr
            addons/zeromq
)

# Is there a better way to find out if catkin_make was run?
if(CATKIN_DEVEL_PREFIX)
  find_package(catkin COMPONENTS roscpp mprl_msgs)
endif()

if (catkin_FOUND)
  catkin_package(CATKIN_DEPENDS roscpp mprl_msgs)
  include_directories(${catkin_INCLUDE_DIRS})
  set(GRL_INCLUDE_DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
  set(GRL_BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  set(GRL_LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  set(GRL_SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else()
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  set(GRL_INCLUDE_DESTINATION include)
  set(GRL_BIN_DESTINATION bin)
  set(GRL_LIB_DESTINATION lib)
  set(GRL_SHARE_DESTINATION share/grl)
endif()

# Build all modules
foreach(module ${MODULES})
  grl_build_library(${module})
endforeach()

install(PROGRAMS bin/grlc bin/grlcc bin/grlq bin/grllib.py bin/tooltip.py DESTINATION ${GRL_BIN_DESTINATION})
install(DIRECTORY cfg/ DESTINATION ${GRL_SHARE_DESTINATION}/cfg FILES_MATCHING PATTERN "*.yaml")

if (catkin_FOUND)
  install(DIRECTORY launch/ DESTINATION ${GRL_SHARE_DESTINATION}/launch FILES_MATCHING PATTERN "*.launch")
endif()
