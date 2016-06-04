# Setup build environment
set(TARGET cppzmq)

FIND_LIBRARY(ZMQ_LIB zmq PATHS /usr/local/lib)
if (NOT ZMQ_LIB)
  message(WARNING "-- ZeroMQ library not found")
else()
  message("-- zmq: ${ZMQ_LIB}")
endif()

if (ZMQ_LIB)
  message("-- Building included CPPZMQ and ZMQ_MESSENGER library")

  # Make library
  add_library(${TARGET} SHARED
              ${SRC}/zmq_messenger.cpp
             )

  target_link_libraries(${TARGET} ${ZMQ_LIB})
  install(TARGETS ${TARGET} DESTINATION lib)
  file(GLOB CPPZMQ_INCLUDES ${SRC}/../include/*)
  install(FILES ${CPPZMQ_INCLUDES} DESTINATION include)
endif()

