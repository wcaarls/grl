# Setup build environment
set(TARGET addon_python)

find_package(PythonLibsNew)

if(PYTHONLIBS_FOUND)
  message("-- Building Python addon")
 
  # Build library
  add_library(${TARGET}
              ${SRC}/python.cpp
             )

  # Add dependencies
  include_directories(${PYTHON_INCLUDE_DIR})
  grl_link_libraries(${TARGET} base)
  target_link_libraries(${TARGET} ${PYTHON_LIBRARIES})
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})

  # Build module
  pybind11_add_module(grlpy
              ${SRC}/grlpy.cpp
             )

  # Add dependencies
  grl_link_libraries(grlpy base)
  set(GRL_PYTHON_DESTINATION ${PYTHON_SITE_PACKAGES} CACHE PATH "grlpy install path")
  install(TARGETS grlpy DESTINATION ${GRL_PYTHON_DESTINATION})
endif()
