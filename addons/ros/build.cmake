# Setup build environment
set(TARGET addon_ros)

if (catkin_FOUND)
  message("-- Building ROS addon")

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
