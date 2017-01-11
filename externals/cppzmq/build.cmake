# Setup build environment
set(TARGET cppzmq)

find_package(ZeroMQ 4.0.0)

if (ZeroMQ_FOUND)
  message("-- Building included CPPZMQ and ZMQ_MESSENGER library")

  # Make library
  add_library(${TARGET} SHARED
              ${SRC}/zmq_messenger.cpp
             )

  include_directories(${ZeroMQ_INCLUDE_DIRS})
  target_link_libraries(${TARGET} ${ZeroMQ_LIBRARIES})
  install(TARGETS ${TARGET} DESTINATION lib)
  file(GLOB CPPZMQ_INCLUDES ${SRC}/../include/*)
  install(FILES ${CPPZMQ_INCLUDES} DESTINATION include)
endif()
