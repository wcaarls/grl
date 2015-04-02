# Setup build environment
set(TARGET addon_rbdl)

add_definitions(-DRBDL_LUA_CONFIG_DIR="${SRC}/../cfg")

# Build library
add_library(${TARGET} SHARED
            ${SRC}/rbdl.cpp
           )

set_source_files_properties(${SRC}/rbdl.cpp PROPERTIES COMPILE_FLAGS "-Wno-pedantic -Wno-variadic-macros")

# Add dependencies
grl_link_libraries(${TARGET} base externals/rbdl)
install(TARGETS ${TARGET} DESTINATION lib/grl)
install(DIRECTORY ${SRC}/../include/grl DESTINATION include FILES_MATCHING PATTERN "*.h")
