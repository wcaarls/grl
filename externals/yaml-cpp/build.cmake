# Setup build environment
set(TARGET yaml-cpp)

message("-- Building included YAML library")

execute_process(
  COMMAND rm -rf externals/yaml-cpp
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

execute_process(
  COMMAND ${CMAKE_COMMAND} -E tar xf ${SRC}/../share/95bb6a18f9523b46499e9b870ef4b8e4d8068fbb.zip
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

execute_process(
  COMMAND mkdir -p externals
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

execute_process(
  COMMAND mv yaml-cpp externals/yaml-cpp
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

execute_process(
  COMMAND cat ${SRC}/../share/fake_map.patch
  COMMAND patch -p1
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/externals/yaml-cpp
)

execute_process(
  COMMAND cat ${SRC}/../share/options.patch
  COMMAND patch -p1
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/externals/yaml-cpp
)

execute_process(
  COMMAND mkdir externals/yaml-cpp/build
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

add_subdirectory(${CMAKE_BINARY_DIR}/externals/yaml-cpp ${CMAKE_BINARY_DIR}/externals/yaml-cpp/build)
