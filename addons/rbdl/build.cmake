# Setup build environment
set(TARGET addon_rbdl)

# Build library
add_library(${TARGET} SHARED
            ${SRC}/rbdl.cpp
           )

# Add dependencies
grl_link_libraries(${TARGET} base externals/rbdl)
install(TARGETS ${TARGET} DESTINATION lib/grl)
install(DIRECTORY ${SRC}/../include/grl DESTINATION include FILES_MATCHING PATTERN "*.h")
