cmake_minimum_required(VERSION 2.8.3)
project(choreonoid_ros_tank)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  choreonoid
  #choreonoid_ros_plugin
  message_generation
)

add_message_files(
  FILES
  JointPosition.msg
)

add_service_files(
  FILES
  SetJointPosition.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS sensor_msgs geometry_msgs roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
)

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
)

function(add_cnoid_simple_controller)
  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)
  add_library(${target} SHARED ${ARGV})
  target_link_libraries(${target} ${catkin_LIBRARIES} ${CHOREONOID_BODY_LIBRARIES})
  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller PREFIX "")
  install(TARGETS ${target} LIBRARY DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller)
endfunction()

add_subdirectory(src)



