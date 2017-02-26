# Setup build environment
set(TARGET addon_llr)

# Build library
add_library(${TARGET} SHARED
            ${SRC}/ertree.cpp
            ${SRC}/ann.cpp
            ${SRC}/llr.cpp
           )

# Add dependencies
grl_link_libraries(${TARGET} base externals/ann)
install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
