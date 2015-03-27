# Setup build environment
set(TARGET addon_glut)

find_package(FreeGLUT)

if (FREEGLUT_FOUND)
  message("-- Building GLUT addon")

  # Build library
  add_library(${TARGET} SHARED
              ${SRC}/glut.cpp
             )

  # Add dependencies
  target_link_libraries(${TARGET} ${FREEGLUT_LIBRARIES})
  grl_link_libraries(${TARGET} base addons/gl)
  install(TARGETS ${TARGET} DESTINATION lib/grl)
  install(DIRECTORY ${SRC}/../include/grl DESTINATION include FILES_MATCHING PATTERN "*.h")
endif()
