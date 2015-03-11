macro(grl_add_mex target)
  # Create mex command
  set(_am_command cd ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${target}.dir && ${MATLAB_BINARY_DIR}/mex -g ${mex_INCLUDE_DIRS} LDFLAGS='$$LDFLAGS -Wl,-rpath,${CMAKE_LIBRARY_OUTPUT_DIRECTORY}' -L${CMAKE_LIBRARY_OUTPUT_DIRECTORY} ${mex_LIBRARIES} -output ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target} ${ARGN})
  add_custom_command(OUTPUT ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target}.mexa64 COMMAND ${_am_command} DEPENDS grl yaml-cpp DEPENDS ${ARGN})
  add_custom_target(${target} ALL DEPENDS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target}.mexa64)
endmacro(grl_add_mex)

# find matlab on path
find_program(MATLAB matlab ENV PATH)
if (NOT ${MATLAB} STREQUAL MATLAB-NOTFOUND)
  execute_process(COMMAND readlink -f ${MATLAB} OUTPUT_VARIABLE MATLAB_EXECUTABLE)
  get_filename_component(MATLAB_BINARY_DIR ${MATLAB_EXECUTABLE} PATH)
  set(MATLAB_INCLUDE_DIR ${MATLAB_BINARY_DIR}/../extern/include)

  message("-- Found Matlab: " ${MATLAB_BINARY_DIR})
endif()

if (MATLAB_INCLUDE_DIR)
  message("-- Building Matlab addon")

  set(mex_INCLUDE_DIRS -I${MATLAB_INCLUDE_DIR} -I${SRC}/../include -I${SRC}/../../../base/include -I${SRC}/../../../externals/yaml-cpp/include)
  set(mex_LIBRARIES    -lgrl -lyaml-cpp -lrt -ldl)

  grl_add_mex(grl_env ${SRC}/convert.cpp ${SRC}/mex_env.cpp)
  grl_add_mex(grl_agent ${SRC}/convert.cpp ${SRC}/mex_agent.cpp)
  install(FILES ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/grl_env.mexa64 ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/grl_agent.mexa64 DESTINATION lib)
endif()
