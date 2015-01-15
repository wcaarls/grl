# Setup build environment
set(TARGET addon_cma)

# Build library
add_library(${TARGET} SHARED
            ${SRC}/cma.cpp
           )

# Add dependencies
grl_link_libraries(${TARGET} base externals/cma)
