# Setup build environment
set(TARGET addon_ros)

if (catkin_FOUND)
  set(GRL_BUILD_ROS ON CACHE BOOL "Build ROS addon")
else()
  message("** Cannot build ROS addon: build not using catkin")
endif()

if (GRL_BUILD_ROS)
  message("** Building ROS addon")

  # Build library
  add_library(${TARGET} SHARED
              ${SRC}/agent.cpp
              ${SRC}/environment.cpp
             )

  # Add dependencies
  target_link_libraries(${TARGET} ${catkin_LIBRARIES})
  grl_link_libraries(${TARGET} base externals/itc)
  add_dependencies(${TARGET} mprl_msgs_generate_messages_cpp)
  install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
  install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
endif()
