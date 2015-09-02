# Setup build environment
set(TARGET addon_leo2)

find_package(PkgConfig)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(FTDIPP libftdipp)

  if (FTDIPP_FOUND)
    message("-- Building LEO/2 addon")

    # Build library
    add_library(${TARGET} SHARED
                ${SRC}/leo2.cpp
               )

    # Add dependencies
    grl_link_libraries(${TARGET} base)
    target_link_libraries(${TARGET} ${FTDIPP_LIBRARIES})
    install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
    install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
  endif()
endif()
