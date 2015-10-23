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

set(TARGETS nmpc_cartpole nmpc_simple)

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
