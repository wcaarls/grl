# Setup build environment
set(TARGET addon_zeromq)

find_package(Protobuf)
find_package(ZeroMQ)

if (PROTOBUF_FOUND AND ZEROMQ_FOUND)
  message("-- Building zeromq addon")

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

  INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS} ${ZEROMQ_INCLUDE_DIR})

  add_dependencies(${TARGET} protobuffer)

  target_link_libraries(${TARGET} ${PROTOBUF_LIBRARIES} ${ZEROMQ_LIBRARIES})
  grl_link_libraries(${TARGET} base externals/cppzmq)
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
  install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
endif()
