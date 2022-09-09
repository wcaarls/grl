# Setup build environment
set(TARGET addon_leosim)

set(WORKSPACE_DIR ${SRC}/../../../externals/odesim)

find_package(PkgConfig)

SET(QT_USE_QTOPENGL TRUE)
find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL)

if (PKG_CONFIG_FOUND AND Qt5OpenGL_DIR)
  pkg_check_modules(TINYXML tinyxml)
  pkg_check_modules(MUPARSER muparser)
  pkg_check_modules(ODE ode)

  if (TINYXML_FOUND AND MUPARSER_FOUND AND ODE_FOUND)
    set(GRL_BUILD_LEOSIM ON CACHE BOOL "Build leosim addon")
  else()
    message("** Cannot build leosim addon: missing one of {tinyxml, muparser, ode}")
  endif()
else()
  message("** Cannot build leosim addon: missing one of {pkgconfig, qt5}")
endif()

if (GRL_BUILD_LEOSIM)
  message("** Building leosim addon")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

  # Build library
  add_library(${TARGET} SHARED
                        ${SRC}/leosim.cpp
                        ${SRC}/LeoBhWalkSym.cpp
                        ${SRC}/STGLeoSim.cpp
                        ${SRC}/ThirdOrderButterworth.cpp)

  include_directories(${SRC}/../include/grl/environments/leosim)

  INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/configuration/configuration.cmake)
  INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/stdlogging.cmake)
  INCLUDE (${WORKSPACE_DIR}/dbl/externals/bithacks/bithacks.cmake)

  add_definitions(-DLEOSIM_CONFIG_DIR="${SRC}/../cfg")

  # Add dependencies
  grl_link_libraries(${TARGET} base addons/odesim)
  target_link_libraries(${TARGET})

  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
  install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
endif()
