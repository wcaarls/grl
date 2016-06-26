# Setup build environment
set(TARGET addon_muscod_models)

FIND_PACKAGE (Lua51)

if (NOT LUA51_FOUND)
  message(WARNING "Cannot build MUSCOD-II models without Lua")
  return()
endif()

# add models
grl_build_library(addons/muscod/models/nmpc_simple)
grl_build_library(addons/muscod/models/nmpc_cartpole)
grl_build_library(addons/muscod/models/mhe_cartpole)
grl_build_library(addons/muscod/models/nmpc_simplest_walker)
#grl_build_library(addons/muscod/models/mhe_simplest_walker)
grl_build_library(addons/muscod/models/nmpc_leo_squat2)
