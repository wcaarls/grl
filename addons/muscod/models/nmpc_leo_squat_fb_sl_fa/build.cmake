# Setup build environment
set(TARGET nmpc_leo_squat_fb_sl_fa)

ADD_LIBRARY ( ${TARGET} SHARED 
              ${SRC}/${TARGET}.cpp
              ${SRC}/leomodel.cpp
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
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/lib${TARGET}.so ${SRC}/../../../cfg/${TARGET}/lib${TARGET}.so)

