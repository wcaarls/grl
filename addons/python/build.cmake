# Setup build environment
set(TARGET grlpy)

find_package(PythonLibsNew)

if(PYTHONLIBS_FOUND)
  message("-- Building Python addon")

  # Build library
  pybind11_add_module(${TARGET} 
              ${SRC}/grlpy.cpp
             )

  # Add dependencies
  grl_link_libraries(${TARGET} base)
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
endif()
