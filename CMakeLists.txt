cmake_minimum_required(VERSION 3.0.2)
project(std_usv)
add_compile_options(-std=c++14 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf tf2
  tf2_geometry_msgs tf2_ros visualization_msgs xmlrpcpp)
find_package(gazebo REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES std_usv_astar std_usv_dwa
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs
    tf tf2 tf2_geometry_msgs tf2_ros visualization_msgs xmlrpcpp)
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(std_usv_astar src/avoidance_astar.cpp)
target_link_libraries(std_usv_astar ${catkin_LIBRARIES})
add_library(std_usv_dwa src/avoidance_dwa.cpp)
target_link_libraries(std_usv_dwa std_usv_astar ${catkin_LIBRARIES})
add_executable(navigation_coordinator_node src/navigation_coordinator_node.cpp)
target_link_libraries(navigation_coordinator_node std_usv_dwa std_usv_astar ${catkin_LIBRARIES})
add_executable(navigation_goal_relay src/navigation_goal_relay.cpp)
target_link_libraries(navigation_goal_relay ${catkin_LIBRARIES})
add_library(std_usv_navigation_sensor_visibility src/navigation_sensor_visibility.cpp)
target_link_libraries(std_usv_navigation_sensor_visibility ${GAZEBO_LIBRARIES})
add_executable(navigation_visual_filter_node src/navigation_visual_filter_node.cpp)
target_link_libraries(navigation_visual_filter_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
catkin_install_python(PROGRAMS
  scripts/generate_navigation_maps.py scripts/navigation_map_publisher.py
  scripts/navigation_cloud_filter.py scripts/thruster_output_adapter.py
  scripts/navigation_dynamic_buoys.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS std_usv_astar std_usv_dwa navigation_coordinator_node navigation_goal_relay
    std_usv_navigation_sensor_visibility navigation_visual_filter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES include/path_planning/avoidance_astar.h include/path_planning/avoidance_dwa.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/path_planning)
install(DIRECTORY launch config maps docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_planning test/test_planning.cpp)
  if(TARGET test_planning)
    target_link_libraries(test_planning std_usv_dwa std_usv_astar ${catkin_LIBRARIES})
  endif()
  add_executable(course_navigation_smoke test/course_navigation_smoke.cpp)
  target_compile_options(course_navigation_smoke PRIVATE -O2)
  target_link_libraries(course_navigation_smoke std_usv_dwa std_usv_astar ${catkin_LIBRARIES})
  catkin_add_nosetests(test/test_map_geometry.py)
  catkin_add_nosetests(test/test_thruster_adapter.py)
  catkin_add_nosetests(test/test_dynamic_buoys.py)
  catkin_add_nosetests(test/test_cloud_filter.py)
  catkin_add_nosetests(test/test_launch_contract.py)
  catkin_add_nosetests(test/test_course_navigation.py DEPENDENCIES course_navigation_smoke TIMEOUT 120)
endif()
