find_package(PkgConfig)
pkg_check_modules(RBDL rbdl>=2.4.0)
 
if (RBDL_FOUND)
  find_library(LUA_ADDON_LIB NAMES rbdl_luamodel HINTS ${RBDL_LIBRARY_DIRS})
  find_path(LUA_ADDON_INC rbdl/addons/luamodel/luamodel.h ${RBDL_INCLUDE_DIRS})

  if (LUA_ADDON_LIB AND LUA_ADDON_INC)
    include_directories(${RBDL_INCLUDE_DIRS})
    target_link_libraries(${TARGET} ${RBDL_LIBRARIES} rbdl_luamodel)
    set_target_properties(${TARGET} PROPERTIES LINK_FLAGS -Wl,-rpath,${RBDL_LIBRARY_DIRS})
    return()
  endif()
endif()

include_directories(${CMAKE_BINARY_DIR}/externals)
include_directories(${CMAKE_BINARY_DIR}/externals/rbdl/include)
include_directories(${CMAKE_BINARY_DIR}/externals/rbdl/build/include)

target_link_libraries(${TARGET} rbdl rbdl_luamodel)
