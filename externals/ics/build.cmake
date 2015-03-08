# Setup build environment
set(TARGET external_ics)

find_package(ZLIB)

if (ZLIB_FOUND)
  message("-- Building ICS external")

  # Make library
  add_library(${TARGET} SHARED
              ${SRC}/libics_binary.c
              ${SRC}/libics_compress.c
              ${SRC}/libics_data.c
              ${SRC}/libics_gzip.c
              ${SRC}/libics_history.c
              ${SRC}/libics_preview.c
              ${SRC}/libics_read.c
              ${SRC}/libics_sensor.c
              ${SRC}/libics_test.c
              ${SRC}/libics_top.c
              ${SRC}/libics_util.c
              ${SRC}/libics_write.c
             )

  target_link_libraries(${TARGET} ${ZLIB_LIBRARIES})
endif()
