# Setup build environment
set(TARGET addon_glut)

find_package(FreeGLUT)

if (FREEGLUT_FOUND)
  set(GRL_BUILD_GLUT ON CACHE BOOL "Build GLUT addon")
else()
  message("** Cannot build GLUT addon: missing FreeGLUT")
endif()

if (GRL_BUILD_GLUT)
  message("** Building GLUT addon")

  # Build library
  add_library(${TARGET} SHARED
              ${SRC}/glut.cpp
             )

  # Add dependencies
  target_link_libraries(${TARGET} ${FREEGLUT_LIBRARIES})
  grl_link_libraries(${TARGET} base addons/gl)
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
  install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
endif()
