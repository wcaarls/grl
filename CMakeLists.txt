project(grl)
cmake_minimum_required(VERSION 2.8.3)
include(grl.cmake)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
  message("-- Set default build type ${CMAKE_BUILD_TYPE}")
endif()

# Choose YAML version
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(LIBYAMLCPP yaml-cpp)
#if (${LIBYAMLCPP_VERSION} VERSION_LESS 0.5)
  message("-- Using included YAML library")
  set(GRL_YAML_MODULE externals/yaml-cpp)
#endif()

# List of modules to build
set(MODULES base
            ${GRL_YAML_MODULE}
            externals/ann
            externals/cma
            externals/ics
            addons/cma
            addons/llr
            addons/glut
   )

# Hack to find out which include directories and libraries
# to pass on to catkin_package
function(include_directories)
  set(INCLUDE_DIRS ${INCLUDE_DIRS} ${ARGN} PARENT_SCOPE)
  _include_directories(${ARGN})
endfunction()

function(target_link_libraries target)
  set(LIBRARIES ${LIBRARIES} ${ARGN} PARENT_SCOPE)
  _target_link_libraries(${target} ${ARGN})
endfunction()

add_custom_target(_grl_dummy)
grl_link_libraries(_grl_dummy ${MODULES})
list(REMOVE_DUPLICATES LIBRARIES)
list(REMOVE_DUPLICATES INCLUDE_DIRS)
list(REMOVE_ITEM INCLUDE_DIRS BEFORE AFTER SYSTEM)

#message("-- grl include directories: ${INCLUDE_DIRS}")
#message("-- grl link libraries: ${LIBRARIES}")

# Is there a better way to find out if catkin_make was run?
if(CATKIN_DEVEL_PREFIX)
  find_package(catkin COMPONENTS roscpp grl_msgs)
endif()

if (catkin_FOUND)
  catkin_package(INCLUDE_DIRS ${INCLUDE_DIRS}
                 LIBRARIES ${LIBRARIES}
                 CATKIN_DEPENDS roscpp grl_msgs
                 DEPENDS eigen
                )
  include_directories(${catkin_INCLUDE_DIRS})
else()
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

# Build all modules
foreach(module ${MODULES})
  grl_build_library(${module})
endforeach()
