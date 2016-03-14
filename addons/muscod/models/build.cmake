# Setup build environment
set(TARGET addon_muscod_models)

pkg_check_modules(RBDL rbdl>=2.4.0)
if (NOT RBDL_FOUND)
  message(WARNING "Cannot build MUSCOD-II models without RBDL")
  return()
endif()

find_library(LUA_ADDON_LIB NAMES rbdl_luamodel HINTS ${RBDL_LIBRARY_DIRS})
find_path(LUA_ADDON_INC rbdl/addons/luamodel/luamodel.h ${RBDL_INCLUDE_DIRS})

if (NOT LUA_ADDON_LIB OR NOT LUA_ADDON_INC)
  message(WARNING "Cannot build MUSCOD-II models without RBDL lua model library")
  return()
endif()

FIND_PACKAGE (PGPLOT)
if (NOT PGPLOT_FOUND)
  message(WARNING "Cannot build MUSCOD-II models without PG Plot")
  return()
endif()

set(TARGETS nmpc_cartpole mhe_cartpole nmpc_simplest_walker mhe_simplest_walker nmpc_simple)

FOREACH (TARGET ${TARGETS})
  ADD_LIBRARY ( ${TARGET} SHARED 
                ${SRC}/${TARGET}.cpp
              )

  TARGET_LINK_LIBRARIES ( ${TARGET}
                          muscod_base
                          ${RBDL_LIBRARIES}
                          ${RBDL_LUAMODEL_LIBRARIES}
                          ${PGPLOT_CPGPLOT_LIBRARY}
                          ${PGPLOT_PGPLOT_LIBRARY}
                        )

  grl_link_libraries(${TARGET} base)
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
  install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
ENDFOREACH(TARGET)

# Cartpole library links
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libnmpc_cartpole.so ${SRC}/../../cfg/inmpc_cartpole/libnmpc_cartpole.so)
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libnmpc_cartpole.so ${SRC}/../../cfg/inmpc_mhe_cartpole/libnmpc_cartpole.so)
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libnmpc_cartpole.so ${SRC}/../../cfg/nmpc_cartpole/libnmpc_cartpole.so)
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libnmpc_cartpole.so ${SRC}/../../cfg/nmpc_mhe_cartpole/libnmpc_cartpole.so)

execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libmhe_cartpole.so ${SRC}/../../cfg/inmpc_mhe_cartpole/libmhe_cartpole.so)
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libmhe_cartpole.so ${SRC}/../../cfg/nmpc_mhe_cartpole/libmhe_cartpole.so)

# Simplest walker library links
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libnmpc_simplest_walker.so ${SRC}/../../cfg/inmpc_simplest_walker/libnmpc_simplest_walker.so)
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libnmpc_simplest_walker.so ${SRC}/../../cfg/inmpc_mhe_simplest_walker/libnmpc_simplest_walker.so)
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libnmpc_simplest_walker.so ${SRC}/../../cfg/nmpc_simplest_walker/libnmpc_simplest_walker.so)
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libnmpc_simplest_walker.so ${SRC}/../../cfg/nmpc_mhe_simplest_walker/libnmpc_simplest_walker.so)

execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libmhe_simplest_walker.so ${SRC}/../../cfg/inmpc_mhe_simplest_walker/libmhe_simplest_walker.so)
execute_process(COMMAND ln -s -f ${CMAKE_CURRENT_BINARY_DIR}/libmhe_simplest_walker.so ${SRC}/../../cfg/nmpc_mhe_simplest_walker/libmhe_simplest_walker.so)

