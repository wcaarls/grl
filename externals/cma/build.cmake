# Setup build environment
set(TARGET cma)

set_source_files_properties(${SRC}/cmaes.c PROPERTIES COMPILE_FLAGS -Wno-unused-result)

# Make library
add_library(${TARGET} SHARED
            ${SRC}/cmaes.c
           )
