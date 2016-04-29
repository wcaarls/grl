# Setup build environment
set(TARGET mhe_simplest_walker)

ADD_LIBRARY ( ${TARGET} SHARED 
              ${SRC}/${TARGET}.cc
              ${SRC}/common_code.cc
              ${SRC}/common_nmpc.cc
            )

TARGET_LINK_LIBRARIES ( ${TARGET}
                        muscod_base
                        ${PGPLOT_CPGPLOT_LIBRARY}
                        ${PGPLOT_PGPLOT_LIBRARY}
                      )

grl_link_libraries(${TARGET} base)
install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")

# Create library links
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libnmpc_simplest_walker.so ${SRC}/../../../cfg/inmpc_mhe_simplest_walker/libnmpc_simplest_walker.so)
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libnmpc_simplest_walker.so ${SRC}/../../../cfg/nmpc_mhe_simplest_walker/libnmpc_simplest_walker.so)
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libmhe_simplest_walker.so ${SRC}/../../../cfg/inmpc_mhe_simplest_walker/libmhe_simplest_walker.so)
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libmhe_simplest_walker.so ${SRC}/../../../cfg/nmpc_mhe_simplest_walker/libmhe_simplest_walker.so)

