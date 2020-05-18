# Setup build environment
set(TARGET addon_tennessee)

include(CMakeDetermineFortranCompiler)
if(CMAKE_Fortran_COMPILER)
  message("** Building Tennessee Eastman addon")

  enable_language(Fortran)

  # Build library
  add_library(${TARGET} SHARED
              ${SRC}/tennessee.cpp
              ${SRC}/teprob.f
              ${SRC}/tectrl.f
             )

  # Add dependencies
  grl_link_libraries(${TARGET} base)
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
  install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
endif()
