# Setup build environment
set(TARGET addon_gl)

find_package(OpenGL)
find_package(ZLIB)

if (OPENGL_FOUND AND ZLIB_FOUND)
  set(GRL_BUILD_GL ON CACHE BOOL "Build OpenGL addon")
else()
  message("** Cannot build OpenGL addon: missing one of {opengl, zlib}")
endif()

if (GRL_BUILD_GL)
  message("** Building OpenGL addon")

  # Build library
  add_library(${TARGET} SHARED
              ${SRC}/sample.cpp
              ${SRC}/random_sample.cpp
              ${SRC}/field.cpp
              ${SRC}/value.cpp
              ${SRC}/mapping.cpp
              ${SRC}/state.cpp
              ${SRC}/trajectory.cpp
              ${SRC}/slice.cpp
              ${SRC}/quadcopter.cpp
             )

  # Add dependencies
  target_link_libraries(${TARGET} ${OPENGL_LIBRARIES})
  grl_link_libraries(${TARGET} base externals/ics externals/pgl)
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
  install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
endif()
