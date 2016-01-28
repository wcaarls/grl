# Setup build environment
set(TARGET addon_lqr)

find_package(PkgConfig)
include(CheckLibraryExists)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(EIGEN3 eigen3)

  if (EIGEN3_FOUND)
    message("-- Building lqr addon")

    check_library_exists(slicot sb02od_ "" HAVE_SLICOT)
    if (HAVE_SLICOT)
      message("-- Using SLICOT to solve LQR DARE")
      add_definitions(-DWITH_SLICOT)
      set(SLICOT_LIBRARIES slicot)
    endif()

    # Build library
    add_library(${TARGET} SHARED
                ${SRC}/lqr.cpp
                ${SRC}/ilqg.cpp
               )

    # Add dependencies
    target_link_libraries(${TARGET} ${SLICOT_LIBRARIES})
    grl_link_libraries(${TARGET} base)
    install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
    install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
  endif()
endif()
