# Setup build environment
set ( TARGET
    passive_nmpc_stages
    # ssiwr_nmpc_stages
)

ADD_LIBRARY ( ${TARGET} SHARED
              ${SRC}/${TARGET}.cc
            )

TARGET_LINK_LIBRARIES ( ${TARGET}
                        muscod_base
                      )

grl_link_libraries(${TARGET} base)
install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")

# Create library links
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/lib${TARGET}.so ${SRC}/../../../cfg/inmpc_simplest_walker/lib${TARGET}.so)
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/lib${TARGET}.so ${SRC}/../../../cfg/nmpc_simplest_walker/lib${TARGET}.so)

