# Setup build environment
set(TARGET cma)

set_source_files_properties(${SRC}/cmaes.c PROPERTIES COMPILE_FLAGS -Wno-unused-result)

# Make library
add_library(${TARGET} SHARED
            ${SRC}/cmaes.c
           )
install(TARGETS ${TARGET} DESTINATION lib)
install(DIRECTORY ${SRC}/../include/cma DESTINATION include FILES_MATCHING PATTERN "*.h")
