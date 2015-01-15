# Setup build environment
set(TARGET addon_llr)

# Build library
add_library(${TARGET} SHARED
            ${SRC}/llr.cpp
           )

# Add dependencies
grl_link_libraries(${TARGET} base externals/ann)
