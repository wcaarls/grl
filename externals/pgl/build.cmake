# Setup build environment
set(TARGET pgl)

execute_process(
  COMMAND rm -rf externals/pgl
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

execute_process(
  COMMAND ${CMAKE_COMMAND} -E tar xf ${SRC}/../share/master.zip
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

execute_process(
  COMMAND mkdir -p externals
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

execute_process(
  COMMAND mv pgl-master externals/pgl
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

execute_process(
  COMMAND mkdir externals/pgl/build
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

add_subdirectory(${CMAKE_BINARY_DIR}/externals/pgl ${CMAKE_BINARY_DIR}/externals/pgl/build)
