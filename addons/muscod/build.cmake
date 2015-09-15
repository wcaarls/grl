# Setup build environment
set(TARGET addon_muscod)

FIND_PACKAGE(MUSCOD)

if (MUSCOD_FOUND)

	pkg_check_modules(RBDL rbdl>=2.4.0)
	if (NOT RBDL_FOUND)
		message(SEND_ERROR "Trying to building MUSCOD-II addon without RBDL")
	endif()

	pkg_check_modules(RBDL_LUAMODEL rbdl_luamodel>=2.4.0)
	if (NOT RBDL_LUAMODEL_FOUND)
		message(SEND_ERROR "RBDL Lua Model library is not found. Perhaps, you have to create rbdl_luamodel.pc manually.")
	endif()

	FIND_PACKAGE ( PGPLOT REQUIRED )
	if (NOT PGPLOT_FOUND)
		message(WARRING "PG Plot is not found.")
	endif()

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
	message("*** MBT: ${MBT}")
	message("*** MUSCOD_DIR: ${MUSCOD_DIR}")
	message("*** MUSCOD_BUILD_TYPE: ${MUSCOD_BUILD_TYPE}")

  # Using some kind of weird build type, bail
  if (NOT EXISTS ${MUSCOD_DIR}/../../Packages/COMMON_CODE/${MUSCOD_BUILD_TYPE})
    message(FATAL_ERROR "Invalid MUSCOD_BUILD_TYPE (" ${MUSCOD_BUILD_TYPE} "), directory does not exist")
  endif()

  message("-- Linking to MUSCOD-II ${MUSCOD_BUILD_TYPE} build")

  include( ${MUSCOD_USE_FILE} )

#  link_directories( ${MUSCOD_DIR}/lib64 )
	link_directories( ${MUSCOD_DIR}/../${MUSCOD_BUILD_TYPE}/lib64 )
  link_directories( ${MUSCOD_DIR}/../../Packages/COMMON_CODE/${MUSCOD_BUILD_TYPE}/lib64/ )
  link_directories( ${MUSCOD_DIR}/../../Packages/LIBLAC/${MUSCOD_BUILD_TYPE}/lib64/ )  
  link_directories( ${MUSCOD_DIR}/../../Packages/INTERFACES/${MUSCOD_BUILD_TYPE}/CPP/ )

  include_directories( ${MUSCOD_DIR}/../../Packages/INTERFACES/${MUSCOD_BUILD_TYPE}/include/ )

  # Build library
  add_library(${TARGET} SHARED
              ${SRC}/nmpc.cpp
							${SRC}/nmpc_th.cpp
             )

  # Add dependencies
  target_link_libraries(${TARGET} muscod_wrapper muscod_base)
  grl_link_libraries(${TARGET} base)
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
  install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")

	################################################################################
	# MUSCOD problems
	set(TARGETS nmpc_cartpole)

	FOREACH (TARGET ${TARGETS})
		ADD_LIBRARY ( ${TARGET} SHARED 
		              ${SRC}/${TARGET}.cpp
		            )

		TARGET_LINK_LIBRARIES ( ${TARGET}
		                        muscod_base
		                        ${RBDL_LIBRARIES}
		                        ${RBDL_LUAMODEL_LIBRARIES}
		                        ${PGPLOT_CPGPLOT_LIBRARY}
		                        ${PGPLOT_PGPLOT_LIBRARY}
		                      )

		grl_link_libraries(${TARGET} base)
		install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
#		install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
	ENDFOREACH(TARGET)
else()
	message(WARNING "-- MUSCOD-II not found")
endif()
