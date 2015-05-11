# Setup build environment
set(TARGET addon_muscod)

FIND_PACKAGE(MUSCOD)

if (MUSCOD_FOUND)
  message("-- Building MUSCOD-II addon")

  set(MUSCOD_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Muscod build output subdirectory")

  include( ${MUSCOD_USE_FILE} )

  link_directories( ${MUSCOD_DIR}/lib64 )
  link_directories( ${MUSCOD_DIR}/../../Packages/COMMON_CODE/${MUSCOD_BUILD_TYPE}/lib64/ )
  link_directories( ${MUSCOD_DIR}/../../Packages/LIBLAC/${MUSCOD_BUILD_TYPE}/lib64/ )  
  link_directories( ${MUSCOD_DIR}/../../Packages/INTERFACES/Debug/CPP/ )

  include_directories( ${MUSCOD_DIR}/../../Packages/INTERFACES/Debug/include/ )

  # Build library
  add_library(${TARGET} SHARED
              ${SRC}/nmpc.cpp
             )

  # Add dependencies
  target_link_libraries(${TARGET} muscod_wrapper muscod_base)
  grl_link_libraries(${TARGET} base)
  install(TARGETS ${TARGET} DESTINATION lib/grl)
  install(DIRECTORY ${SRC}/../include/grl DESTINATION include FILES_MATCHING PATTERN "*.h")
endif()
