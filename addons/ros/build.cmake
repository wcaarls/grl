# Setup build environment
set(TARGET addon_ros)

if (catkin_FOUND)
  # Build library
  add_library(${TARGET} SHARED
              ${SRC}/agent.cpp
              ${SRC}/environment.cpp
             )

  # Add dependencies
  target_link_libraries(${TARGET} ${catkin_LIBRARIES})
  grl_link_libraries(${TARGET} base externals/itc)
  add_dependencies(${TARGET} mprl_msgs_generate_messages_cpp)
  install(TARGETS ${TARGET} DESTINATION lib/grl)
  install(DIRECTORY ${SRC}/../include/grl DESTINATION include)
endif()
