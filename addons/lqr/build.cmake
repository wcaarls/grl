# Setup build environment
set(TARGET addon_lqr)

include(CheckLibraryExists)

check_library_exists(slicot sb02od_ "" HAVE_SLICOT)
if (HAVE_SLICOT)
  message("-- Using SLICOT to solve LQR DARE")
  add_definitions(-DWITH_SLICOT)
  set(SLICOT_LIBRARIES slicot)
endif()

# Build library
add_library(${TARGET} SHARED
            ${SRC}/lqr.cpp
            ${SRC}/ilqg.cpp
           )

# Add dependencies
target_link_libraries(${TARGET} ${SLICOT_LIBRARIES})
grl_link_libraries(${TARGET} base)
install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
