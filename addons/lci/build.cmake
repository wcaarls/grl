# Setup build environment
set(TARGET addon_lci)

message("** Building LCI addon")

# Build library
add_library(${TARGET} SHARED
            ${SRC}/serial.cpp
            ${SRC}/cartpole.cpp
           )

# Add dependencies
include_directories(${FTDIPP_INCLUDE_DIRS})
grl_link_libraries(${TARGET} base)
target_link_libraries(${TARGET} ${FTDIPP_LIBRARIES})
install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
