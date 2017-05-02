cmake_minimum_required(VERSION 2.8.3)
project(cr_mit)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES CR_mit
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(marker_viz src/marker_viz.cpp)
target_link_libraries(marker_viz 
   ${catkin_LIBRARIES})
add_dependencies(marker_viz ${${cr_mit}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(spuit_control src/spuit_control.cpp)
target_link_libraries(spuit_control
   ${catkin_LIBRARIES})
add_dependencies(spuit_control ${${cr_mit}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(surface_visualisation src/surface_visualisation.cpp)
target_link_libraries(surface_visualisation
   ${catkin_LIBRARIES})
add_dependencies(surface_visualisation ${${cr_mit}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(clean_state src/clean_state.cpp)
target_link_libraries(clean_state
   ${catkin_LIBRARIES})
add_dependencies(clean_state ${${cr_mit}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

