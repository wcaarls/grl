# Setup build environment
set(TARGET addon_muscod_models)

find_library(LUA_ADDON_LIB NAMES rbdl_luamodel HINTS ${RBDL_LIBRARY_DIRS})
find_path(LUA_ADDON_INC rbdl/addons/luamodel/luamodel.h ${RBDL_INCLUDE_DIRS})

if (NOT LUA_ADDON_LIB OR NOT LUA_ADDON_INC)
  message(WARNING "Cannot build MUSCOD-II models without RBDL lua model library")
  return()
endif()

# add models
grl_build_library(addons/muscod/models/nmpc_simple)
grl_build_library(addons/muscod/models/nmpc_cartpole)
grl_build_library(addons/muscod/models/mhe_cartpole)
grl_build_library(addons/muscod/models/nmpc_simplest_walker)
#grl_build_library(addons/muscod/models/mhe_simplest_walker)
