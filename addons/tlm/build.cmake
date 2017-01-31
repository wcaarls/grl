# Setup build environment
set(TARGET addon_tlm)

# Build library
add_library(${TARGET} SHARED
            ${SRC}/tlm.cpp
           )

# Add dependencies
grl_link_libraries(${TARGET} base)
install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
