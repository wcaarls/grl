# Setup build environment
set(TARGET addon_tensorflow)

add_custom_command(OUTPUT ${SRC}/../lib COMMAND ln -s ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} ${SRC}/../lib)
add_custom_target(${TARGET} ALL DEPENDS ${SRC}/../lib)

install(FILES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/lib${TARGET}.so DESTINATION ${GRL_LIB_DESTINATION} OPTIONAL)
