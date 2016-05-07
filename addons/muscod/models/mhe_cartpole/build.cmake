# Setup build environment
set(TARGET mhe_cartpole)

ADD_LIBRARY ( ${TARGET} SHARED 
              ${SRC}/${TARGET}.cpp
            )

TARGET_LINK_LIBRARIES ( ${TARGET}
                        muscod_base
                        ${PGPLOT_CPGPLOT_LIBRARY}
                        ${PGPLOT_PGPLOT_LIBRARY}
                      )

grl_link_libraries(${TARGET} base externals/rbdl)
install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")

# Cartpole library links
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libnmpc_cartpole.so ${SRC}/../../../cfg/inmpc_mhe_cartpole/libnmpc_cartpole.so)
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libnmpc_cartpole.so ${SRC}/../../../cfg/nmpc_mhe_cartpole/libnmpc_cartpole.so)
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libmhe_cartpole.so ${SRC}/../../../cfg/inmpc_mhe_cartpole/libmhe_cartpole.so)
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libmhe_cartpole.so ${SRC}/../../../cfg/nmpc_mhe_cartpole/libmhe_cartpole.so)

