# Setup build environment
set(TARGET addon_box2d)

FIND_PACKAGE (Box2D)

if (BOX2D_FOUND)
  set(GRL_BUILD_BOX2D ON CACHE BOOL "Build Box2D addon")
else()
  message("** Cannot build Box2D addon: missing box2d")
endif()
  
if (GRL_BUILD_BOX2D)
  message("** Building Box2D addon")

  include_directories(${BOX2D_INCLUDE_DIR})

  # Build library
  add_library(${TARGET} SHARED
              ${SRC}/breakout.cpp
             )

  # Add dependencies
  grl_link_libraries(${TARGET} base)
  target_link_libraries(${TARGET} ${BOX2D_LIBRARIES})
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
  install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
endif()
