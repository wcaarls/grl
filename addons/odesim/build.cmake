# Setup build environment
set(TARGET addon_odesim)

set(WORKSPACE_DIR ${SRC}/../../../externals/odesim)

find_package(PkgConfig)

SET(QT_USE_QTOPENGL TRUE)
find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL)

if (PKG_CONFIG_FOUND AND Qt5OpenGL_DIR)
  pkg_check_modules(TINYXML tinyxml)
  pkg_check_modules(MUPARSER muparser)
  pkg_check_modules(ODE ode)

  if (TINYXML_FOUND AND MUPARSER_FOUND AND ODE_FOUND)
    set(GRL_BUILD_ODESIM ON CACHE BOOL "Build odesim addon")
  else()
    message("** Cannot build odesim addon: missing one of {tinyxml, muparser, ode}")
  endif()
else()
  message("** Cannot build odesim addon: missing one of {pkgconfig, qt5}")
endif()

if (GRL_BUILD_ODESIM)
  message("** Building odesim addon")

  # ODE floating point precision definition
  ADD_DEFINITIONS(-DdDOUBLE -DODESIM_TEXTURE_DIR="${SRC}/../textures")

  INCLUDE_DIRECTORIES(${TINYXML_INCLUDE_DIRS} ${MUPARSER_INCLUDE_DIRS})

  SET(QT_FORMS_UI ${SRC}/../ui/odesim_dialog.ui)
  SET(QT_MOC_HDRS ${SRC}/../include/grl/environments/odesim/dialog.h ${SRC}/../include/grl/environments/odesim/environment.h)

  QT5_WRAP_UI(QT_FORMS_ODESIM ${QT_FORMS_UI})
  QT5_WRAP_CPP(QT_MOC_ODESIM  ${QT_MOC_HDRS})

  get_filename_component(QT_FORMS_INCLUDE_DIR ${QT_FORMS_ODESIM} PATH)
  INCLUDE_DIRECTORIES(${QT_FORMS_INCLUDE_DIR})

  ADD_DEFINITIONS(-DCONFIG_DIR="${SRC}/../cfg")

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

  INCLUDE(${WORKSPACE_DIR}/dbl/platform/include.cmake)

  # Build library
  add_library(${TARGET} SHARED
                        ${SRC}/dialog.cpp ${SRC}/environment.cpp ${SRC}/simulator.cpp
                        ${QT_MOC_ODESIM} ${QT_FORMS_ODESIM})

  INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/configuration/configuration.cmake)
  INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/stdlogging.cmake)
  INCLUDE (${WORKSPACE_DIR}/dbl/platform/simulation/odesim/odesim.cmake)

  # Add dependencies
  grl_link_libraries(${TARGET} base)
  target_link_libraries(${TARGET} ${QT_LIBRARIES} ${TINYXML_LIBRARIES} ${MUPARSER_LIBRARIES} ${ODE_LIBRARIES} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL)
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
  install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
endif()

