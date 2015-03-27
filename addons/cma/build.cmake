# Setup build environment
set(TARGET addon_cma)

# Build library
add_library(${TARGET} SHARED
            ${SRC}/cma.cpp
           )

# Add dependencies
grl_link_libraries(${TARGET} base externals/cma)
install(TARGETS ${TARGET} DESTINATION lib/grl)
install(DIRECTORY ${SRC}/../include/grl DESTINATION include FILES_MATCHING PATTERN "*.h")
