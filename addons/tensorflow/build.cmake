find_package(Protobuf)
check_library_exists(tensorflow TF_Version "" TENSORFLOW_FOUND)
find_path(TENSORFLOW_INCLUDE_DIRS tensorflow/c/c_api.h)

if (PROTOBUF_FOUND AND TENSORFLOW_FOUND)
  message("-- Building TensorFlow addon")

  # Build stub
  set(TARGET addon_tensorflow)
  add_library(${TARGET} SHARED
              ${SRC}/stub.cpp
             )
  grl_link_libraries(${TARGET} base)
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})

  # Generate protobuf headers
  protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS ${SRC}/../share/graph.proto)

  # Build library
  set(TARGET 2addon_tensorflow)
  add_library(${TARGET} SHARED
              ${PROTO_SOURCES}
              ${SRC}/tensorflow.cpp
              ${SRC}/naf.cpp
             )

  # Add dependencies
  target_include_directories(${TARGET} PUBLIC ${PROTOBUF_INCLUDE_DIRS} ${TENSORFLOW_INCLUDE_DIRS} ${CMAKE_BINARY_DIR})
  target_link_libraries(${TARGET} ${PROTOBUF_LIBRARIES})
  grl_link_libraries(${TARGET} base)
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
  install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
endif()
