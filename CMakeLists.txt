project(grl)
cmake_minimum_required(VERSION 2.8.3)
include(grl.cmake)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
  message("-- Set default build type ${CMAKE_BUILD_TYPE}")
endif()

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -Wno-unused-parameter)

# Choose YAML version
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(LIBYAMLCPP yaml-cpp)
#if (${LIBYAMLCPP_VERSION} VERSION_LESS 0.5)
  message("-- Using included YAML library")
  set(GRL_YAML_MODULE externals/yaml-cpp)
#endif()

if (WIN32)
  set(GRL_WIN32_MODULE externals/win32)
endif()

# List of modules to build
set(MODULES base
            ${GRL_WIN32_MODULE}
            ${GRL_YAML_MODULE}
            externals/ann
            externals/cma
            externals/ics
            externals/itc
            addons/cma
            addons/llr
            addons/gl
            addons/glut
            addons/ros
   )

# Is there a better way to find out if catkin_make was run?
if(CATKIN_DEVEL_PREFIX)
  find_package(catkin COMPONENTS roscpp mprl_msgs)
endif()

if (catkin_FOUND)
  catkin_package(CATKIN_DEPENDS roscpp mprl_msgs)
  include_directories(${catkin_INCLUDE_DIRS})
else()
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

# Build all modules
foreach(module ${MODULES})
  grl_build_library(${module})
endforeach()
