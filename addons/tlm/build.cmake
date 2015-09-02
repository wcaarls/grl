# Setup build environment
set(TARGET addon_tlm)

find_package(PkgConfig)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(EIGEN3 eigen3)

  if (EIGEN3_FOUND)
    message("-- Building tlm addon")

    # Build library
    add_library(${TARGET} SHARED
                ${SRC}/tlm.cpp
               )

    # Add dependencies
    grl_link_libraries(${TARGET} base)
    install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
    install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
  endif()
endif()
