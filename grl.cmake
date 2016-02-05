set(GRL_DIR ${CMAKE_CURRENT_LIST_DIR})

# Find directory of a grl module
macro(grl_get_module_dir module dir)
  if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${module}/build.cmake")
    set(${dir} ${CMAKE_CURRENT_SOURCE_DIR}/${module})
  else()
    set(${dir} ${GRL_DIR}/${module})
  endif()
endmacro(grl_get_module_dir)

# Specify grl modules a target depends on
macro(grl_link_libraries target)
  set(${target}_target ${TARGET}) 
  set(TARGET ${target})
  foreach(_dep ${ARGN})
    grl_get_module_dir(${_dep} _depdir)

    if (EXISTS "${_depdir}/include")
      include_directories(${_depdir}/include)
    endif()
    
    include(${_depdir}/link.cmake)
  endforeach()
  set(TARGET ${${target}_target})
endmacro(grl_link_libraries)

# Build a target
macro(grl_build_library target)
  grl_get_module_dir(${target} _targetdir)

  set(SRC ${_targetdir}/src)
  if (EXISTS "${_targetdir}/cmake")
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${_targetdir}/cmake)
  endif()
  if (EXISTS "${_targetdir}/include")
    include_directories(${_targetdir}/include)
  endif()
  include(${_targetdir}/build.cmake)
endmacro(grl_build_library)
