# Setup build environment
set(TARGET nmpc_simple)

ADD_LIBRARY ( ${TARGET} SHARED 
              ${SRC}/${TARGET}.cpp
            )

TARGET_LINK_LIBRARIES ( ${TARGET}
                        muscod_base
                        ${PGPLOT_CPGPLOT_LIBRARY}
                        ${PGPLOT_PGPLOT_LIBRARY}
                      )

grl_link_libraries(${TARGET} base)
install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")

