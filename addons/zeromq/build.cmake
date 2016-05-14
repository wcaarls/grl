# Setup build environment
set(TARGET addon_zeromq)

FIND_LIBRARY(ZMQ_LIB zmq)
if (NOT ZMQ_LIB)
  message(WARNING "-- ZeroMQ library not found")
endif()

set(PROTOBUF_FOUND FALSE)
find_package(Protobuf)
if (NOT ${PROTOBUF_FOUND})
  message(WARNING "-- Google Protocol Buffers library not found")
endif()

if (ZMQ_LIB AND ${PROTOBUF_FOUND})
  message("-- Building ZeroMQ addon")

  add_custom_command(
                      OUTPUT  ${CMAKE_CURRENT_BINARY_DIR}/drl_messages.pb.cc ${CMAKE_CURRENT_BINARY_DIR}/drl_messages.pb.h
                      COMMAND protoc -I=${SRC}/../protobuffer/ --cpp_out=${CMAKE_CURRENT_BINARY_DIR} ${SRC}/../protobuffer/drl_messages.proto
    )

  add_custom_target(protobuffer ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/drl_messages.pb.cc ${CMAKE_CURRENT_BINARY_DIR}/drl_messages.pb.h)

  # Build library
  add_library(${TARGET} SHARED
              ${SRC}/zeromq.cpp
              ${CMAKE_CURRENT_BINARY_DIR}/drl_messages.pb.cc
             )

  INCLUDE_DIRECTORIES(${SRC}/../../../externals/cppzmq/include/)

  add_dependencies(${TARGET} protobuffer)

  target_link_libraries(${TARGET} ${ZMQ_LIB} ${PROTOBUF_LIBRARY})
  grl_link_libraries(${TARGET} base)
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
  install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")

endif()
