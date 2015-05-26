# Setup build environment
set(TARGET addon_cma)

# Build library
add_library(${TARGET} SHARED
            ${SRC}/cma.cpp
           )

# Add dependencies
grl_link_libraries(${TARGET} base externals/cma)
install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
