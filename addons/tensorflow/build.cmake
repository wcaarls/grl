# Setup build environment
set(TARGET addon_tensorflow)

find_package(Protobuf)
find_path(TENSORFLOW_INCLUDE_DIRS tensorflow/c/c_api.h)

if (PROTOBUF_FOUND AND TENSORFLOW_INCLUDE_DIRS)
  set(GRL_BUILD_TENSORFLOW ON CACHE BOOL "Build TensorFlow addon")
else()
  message("** Cannot build TensorFlow addon: missing one of {protobuf, tensorflow-c-api}")
endif()

if (GRL_BUILD_TENSORFLOW)
  message("** Building TensorFlow addon")

  # Generate protobuf headers
  protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS ${SRC}/../share/graph.proto ${SRC}/../share/config.proto)

  # Build library
  add_library(${TARGET} SHARED
              ${SRC}/tensorflow_api.cpp
              ${PROTO_SOURCES}
              ${SRC}/tensorflow.cpp
              ${SRC}/naf.cpp
              ${SRC}/ddpg.cpp
             )

  # Add dependencies
  target_include_directories(${TARGET} PUBLIC ${PROTOBUF_INCLUDE_DIRS} ${TENSORFLOW_INCLUDE_DIRS} ${CMAKE_BINARY_DIR})
  target_link_libraries(${TARGET} ${PROTOBUF_LIBRARIES})
  grl_link_libraries(${TARGET} base)
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
  install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
endif()
