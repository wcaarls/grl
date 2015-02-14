# Specify grl modules a target depends on
macro(grl_link_libraries target)
  set(${target}_target ${TARGET}) 
  set(TARGET ${target})
  foreach(_dep ${ARGN})
    if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_dep}/include")
      include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${_dep}/include)
    endif()
    include(${CMAKE_CURRENT_SOURCE_DIR}/${_dep}/link.cmake)
  endforeach()
  set(TARGET ${${target}_target})
endmacro(grl_link_libraries)

# Build a target
macro(grl_build_library target)
  set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/${target}/src)
  if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${target}/cmake")
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/${target}/cmake)
  endif()
  if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${target}/include")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${target}/include)
  endif()
  include(${CMAKE_CURRENT_SOURCE_DIR}/${target}/build.cmake)
endmacro(grl_build_library)

# Link to correct YAML
macro(grl_link_yaml target)
  if (DEFINED GRL_YAML_MODULE)
    grl_link_libraries(${target} externals/yaml-cpp)
  else()
    target_link_libraries(${target} -lyaml-cpp)
  endif()
endmacro(grl_link_yaml)

# Find Matlab
macro(find_matlab_path)
  if (NOT MATLAB_INCLUDE_DIR)
    # find matlab on path
    find_program(MATLAB matlab ENV PATH)
    if (NOT ${MATLAB} STREQUAL MATLAB-NOTFOUND)
      execute_process(COMMAND readlink -f ${MATLAB} OUTPUT_VARIABLE MATLAB_EXECUTABLE)
      get_filename_component(MATLAB_BINARY_DIR ${MATLAB_EXECUTABLE} PATH)
      set(MATLAB_INCLUDE_DIR ${MATLAB_BINARY_DIR}/../extern/include)
    endif()
  endif()
endmacro()

# Add a mex library target
macro(grl_add_mex target)
  # Find matlab
  find_matlab_path()

  if (MATLAB_INCLUDE_DIR)
    # Find include directories
    set(mex_INCLUDE_DIRS ${INCLUDE_DIRS})
    list(REMOVE_DUPLICATES mex_INCLUDE_DIRS)
    list(REMOVE_ITEM mex_INCLUDE_DIRS BEFORE AFTER SYSTEM)
    set(mex_INCLUDE_CFLAGS)
    foreach(id ${mex_INCLUDE_DIRS})
      set(mex_INCLUDE_CFLAGS ${mex_INCLUDE_CFLAGS} -I${id})
    endforeach()

    # Create mex command
    set(_am_command cd ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${target}.dir && ${MATLAB_BINARY_DIR}/mex -g -I${MATLAB_INCLUDE_DIR} -I${CMAKE_CURRENT_SOURCE_DIR}/grl/grl_matlab/include ${mex_INCLUDE_CFLAGS} ${MPRL_DEFINITIONS} LDFLAGS='$$LDFLAGS -Wl,-rpath,${CMAKE_LIBRARY_OUTPUT_DIRECTORY}' -L${CMAKE_LIBRARY_OUTPUT_DIRECTORY} -l${target} -lgrl_common -lgrl_matlab -lrt -output ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target} ${ARGN})
    add_custom_command(OUTPUT ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target}.mexa64 COMMAND ${_am_command} DEPENDS ${target} grl_common grl_matlab)
    add_custom_target(${target}_mex ALL DEPENDS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target}.mexa64)
  else()
    message("-- Matlab not found, skipping mex file generation for ${target}")
  endif()
endmacro(grl_add_mex)
