# Setup build environment
set(TARGET yaml-cpp)

find_package(PkgConfig)
pkg_check_modules(YAML yaml-cpp)

if (YAML_FOUND)
	message("-- Using external YAML library")
	return()
endif()

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

if (false)
# export yaml-cpp module path and check if pkg-config can find it
#set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CMAKE_BINARY_DIR}/externals/yaml-cpp")

pkg_check_modules(RBDL yaml-cpp>=0.5.2)

if (RBDL_FOUND)
	message("-- Building included RBDL library")
else()
	message(WARNING "-- RBDL library is not found")
endif()

endif()
