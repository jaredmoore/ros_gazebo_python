cmake_minimum_required(VERSION 2.8.3)
project(world_step)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_ros
  message_generation
)

find_package(gazebo REQUIRED)

add_service_files(
   FILES
   step_world.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(step_world_server src/step_world.cpp)
target_link_libraries(step_world_server ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(step_world_server world_step_gencpp)
