# Setup build environment
set(TARGET addon_muscod)

FIND_PACKAGE(MUSCOD)

if (MUSCOD_FOUND)

	FIND_PACKAGE ( PGPLOT )
	if (NOT PGPLOT_FOUND)
		message(WARNING "Cannot build MUSCOD-II addon without PG Plot")
		return()
	endif()

	add_definitions(-DMUSCOD_CONFIG_DIR="${SRC}/../cfg")

  # Find preferred Muscod build type
  if (${CMAKE_BUILD_TYPE} MATCHES "Debug")
    set(MBT Debug)
  elseif (${CMAKE_BUILD_TYPE} MATCHES "RelWithDebInfo|Release")
    set(MBT Release)
  endif()

  # If preferred not available, choose something that is
  if (NOT EXISTS ${MUSCOD_DIR}/../../Packages/COMMON_CODE/${MBT})
    unset(MBT)
    foreach(BT Release Debug)
      if (EXISTS ${MUSCOD_DIR}/../../Packages/COMMON_CODE/${BT})
        set(MBT ${BT})
      endif()
    endforeach()
  endif()

  #set(MUSCOD_BUILD_TYPE ${MBT} CACHE STRING "Muscod build output subdirectory")
  set(MUSCOD_BUILD_TYPE ${MBT})

  # Using some kind of weird build type, bail
  if (NOT EXISTS ${MUSCOD_DIR}/../../Packages/COMMON_CODE/${MUSCOD_BUILD_TYPE})
    message(FATAL_ERROR "Invalid MUSCOD_BUILD_TYPE (" ${MUSCOD_BUILD_TYPE} "), directory does not exist")
  endif()

  message("-- Linking to MUSCOD-II ${MUSCOD_BUILD_TYPE} build")

  include( ${MUSCOD_USE_FILE} )

  link_directories( ${MUSCOD_DIR}/lib64 )
  link_directories( ${MUSCOD_DIR}/../../Packages/COMMON_CODE/${MUSCOD_BUILD_TYPE}/lib64/ )
  link_directories( ${MUSCOD_DIR}/../../Packages/LIBLAC/${MUSCOD_BUILD_TYPE}/lib64/ )
  link_directories( ${MUSCOD_DIR}/../../Packages/INTERFACES/${MUSCOD_BUILD_TYPE}/CPP/ )

  include_directories( ${MUSCOD_DIR}/../../Packages/INTERFACES/${MUSCOD_BUILD_TYPE}/include/ )

  # Build library
  add_library(${TARGET} SHARED
              ${SRC}/nmpc_base.cpp
              ${SRC}/nmpc.cpp
              ${SRC}/mhe_nmpc.cpp
              ${SRC}/nmpc_th.cpp
              ${SRC}/nmpc_mlrti.cpp
             )

  # Add dependencies
  target_link_libraries(${TARGET} muscod_wrapper muscod_base)
  grl_link_libraries(${TARGET} base)
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
  install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")

  # Build models
  grl_build_library(addons/muscod/models)
else()
  message(WARNING "-- MUSCOD-II not found")
endif()
