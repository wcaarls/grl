###
### CMake settings
###

include(CheckCXXCompilerFlag)

###
### Project settings
###

set(YAML_CPP_VERSION_MAJOR "0")
set(YAML_CPP_VERSION_MINOR "5")
set(YAML_CPP_VERSION_PATCH "1")
set(YAML_CPP_VERSION "${YAML_CPP_VERSION_MAJOR}.${YAML_CPP_VERSION_MINOR}.${YAML_CPP_VERSION_PATCH}")

###
### Project options
###
## Project stuff
option(YAML_CPP_BUILD_TOOLS "Enable testing and parse tools" ON)
option(YAML_CPP_BUILD_CONTRIB "Enable contrib stuff in library" ON)

## Build options
# --> General
# see http://www.cmake.org/cmake/help/cmake2.6docs.html#variable:BUILD_SHARED_LIBS
#     http://www.cmake.org/cmake/help/cmake2.6docs.html#command:add_library
option(BUILD_SHARED_LIBS "Build Shared Libraries" ON)

###
### Sources, headers, directories and libs
###
set(header_directory "${SRC}/../include/yaml-cpp/")

file(GLOB sources "${SRC}/[a-zA-Z]*.cpp")
file(GLOB_RECURSE public_headers "include/yaml-cpp/[a-zA-Z]*.h")
file(GLOB private_headers "src/[a-zA-Z]*.h")

if(YAML_CPP_BUILD_CONTRIB)
	file(GLOB contrib_sources "${SRC}/contrib/[a-zA-Z]*.cpp")
	file(GLOB contrib_public_headers "${SRC}/../include/yaml-cpp/contrib/[a-zA-Z]*.h")
	file(GLOB contrib_private_headers "${SRC}/contrib/[a-zA-Z]*.h")
else()
	add_definitions(-DYAML_CPP_NO_CONTRIB)
endif()

if(VERBOSE)
	message(STATUS "sources: ${sources}")
	message(STATUS "public_headers: ${public_headers}")
	message(STATUS "private_headers: ${private_headers}")
	message(STATUS "contrib_sources: ${contrib_sources}")
	message(STATUS "contrib_public_headers: ${contrib_public_headers}")
	message(STATUS "contrib_private_headers: ${contrib_private_headers}")
endif()

include_directories(${YAML_CPP_SOURCE_DIR}/src)
include_directories(${YAML_CPP_SOURCE_DIR}/include)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

###
### General compilation settings
###
if(BUILD_SHARED_LIBS)
	set(LABEL_SUFFIX "shared")
else()
	set(LABEL_SUFFIX "static")
endif()

# GCC specialities
if(CMAKE_COMPILER_IS_GNUCXX)
	### General stuff
	if(WIN32)
		set(CMAKE_SHARED_LIBRARY_PREFIX "")	# DLLs do not have a "lib" prefix
		set(CMAKE_IMPORT_LIBRARY_PREFIX "")	# same for DLL import libs
		set(CMAKE_LINK_DEF_FILE_FLAG "")	# CMake workaround (2.8.3)
	endif()

	### Project stuff
	if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
		set(CMAKE_BUILD_TYPE Release)
	endif()
	#
	set(CMAKE_CXX_FLAGS_RELEASE "-O2")
	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")
	set(CMAKE_CXX_FLAGS_DEBUG "-g")
	set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os")
	#
	set(GCC_EXTRA_OPTIONS "")
	#
	set(FLAG_TESTED "-Wextra")
	check_cxx_compiler_flag(${FLAG_TESTED} FLAG_WEXTRA)
	if(FLAG_WEXTRA)
		set(GCC_EXTRA_OPTIONS "${GCC_EXTRA_OPTIONS} ${FLAG_TESTED}")
	endif()
	#
	set(CMAKE_CXX_FLAGS "-Wall ${GCC_EXTRA_OPTIONS} -pedantic -Wno-long-long ${CMAKE_CXX_FLAGS}")
	#
	add_custom_target(debuggable $(MAKE) clean
		COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Debug ${CMAKE_SOURCE_DIR}
		COMMENT "Adjusting settings for debug compilation"
		VERBATIM)
	add_custom_target(releasable $(MAKE) clean
		COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Release ${CMAKE_SOURCE_DIR}
		COMMENT "Adjusting settings for release compilation"
		VERBATIM)
endif()

###
### Library
###
add_library(yaml-cpp
	${sources}
	${public_headers}
	${private_headers}
	${contrib_sources}
	${contrib_public_headers}
	${contrib_private_headers}
)
