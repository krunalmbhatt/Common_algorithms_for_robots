cmake_minimum_required(VERSION 3.0.2)
project(all_algo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
)

## Find OpenCV
find_package(OpenCV REQUIRED)

catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library for collision_check
add_library(collision_check_lib src/collision_check.cpp)
target_link_libraries(collision_check_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Declare a C++ executable for each algorithm
add_executable(astar src/astar.cpp)
target_link_libraries(astar ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} collision_check_lib)

add_executable(bfs src/bfs.cpp)
target_link_libraries(bfs ${catkin_LIBRARIES})

add_executable(dfs src/dfs.cpp)
target_link_libraries(dfs ${catkin_LIBRARIES})

add_executable(dijkstra src/dijkstra.cpp)
target_link_libraries(dijkstra ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} collision_check_lib)

add_executable(rrt src/rrt.cpp)
target_link_libraries(rrt ${catkin_LIBRARIES})

add_executable(prm src/prm.cpp)
target_link_libraries(prm ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} collision_check_lib)

add_executable(rrt_star src/rrt_star.cpp)
target_link_libraries(rrt_star ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} collision_check_lib)

## Install executables
install(TARGETS astar bfs dfs dijkstra rrt rrt_star prm
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_all_algo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
