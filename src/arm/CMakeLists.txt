cmake_minimum_required(VERSION 3.0.2)
project(arm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_planning_interface
)


catkin_package(
  CATKIN_DEPENDS
    roscpp
    moveit_ros_planning_interface
)

include_directories(${catkin_INCLUDE_DIRS})

add_compile_options(-std=c++11)

add_executable(simple_move src/simple_move.cpp)
add_dependencies(simple_move ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_move ${catkin_LIBRARIES})

add_executable(pick_place_tutorial src/pick_place_tutorial.cpp)
target_link_libraries(pick_place_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pick_place_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(simple_grasp src/simple_grasp.cpp)
target_link_libraries(simple_grasp ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS simple_grasp DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

