# Setup build environment
set(TARGET addon_zeromq)

find_package(ZeroMQ 4.0.0)
find_package(Protobuf)

if (ZeroMQ_FOUND AND PROTOBUF_FOUND)
  set(GRL_BUILD_ZEROMQ ON CACHE BOOL "Build ZeroMQ addon")
else()
  message("** Cannot build ZeroMW addon: missing one of {zeromq, protobuf}")
endif()

if (GRL_BUILD_ZEROMQ)
  message("** Building ZeroMQ addon")

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

  add_dependencies(${TARGET} protobuffer)

  target_link_libraries(${TARGET} ${PROTOBUF_LIBRARY})
  grl_link_libraries(${TARGET} base externals/cppzmq)
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
  install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")

endif()
