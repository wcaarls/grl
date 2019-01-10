# Setup build environment
set(TARGET addon_rbdl)

FIND_PACKAGE (Lua51)

if (LUA51_FOUND)
  set(GRL_BUILD_RBDL ON CACHE BOOL "Build RBDL addon")
else()
  message("** Cannot build RBDL addon: missing lua")
endif()
  
if (GRL_BUILD_RBDL)
  message("** Building RBDL addon")

  add_definitions(-DRBDL_LUA_CONFIG_DIR="${SRC}/../cfg")

  include_directories(${LUA_INCLUDE_DIR})

  # Build library
  add_library(${TARGET} SHARED
              ${SRC}/rbdl.cpp
#              ${SRC}/leohelper.cpp
              ${SRC}/leo/rbdl_leo_task.cpp
              ${SRC}/leo/rbdl_leo_model.cpp
             )

  set_source_files_properties(${SRC}/rbdl.cpp PROPERTIES COMPILE_FLAGS "-Wno-pedantic -Wno-variadic-macros")

  # Add dependencies
  grl_link_libraries(${TARGET} base externals/rbdl)
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
  install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
endif()
