cmake_minimum_required(VERSION 3.0.2)
project(auto_drive)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  visualization_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  uuid_msgs
  nav_msgs
  carla_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

add_message_files(
  DIRECTORY waypoint_msgs 
  FILES 
  Waypoint.msg 
  WaypointArray.msg
)

generate_messages(
  DEPENDENCIES 
  geometry_msgs 
  std_msgs
  sensor_msgs 
  uuid_msgs 
  nav_msgs 
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy roscpp std_msgs visualization_msgs
  message_runtime roscpp geometry_msgs std_msgs uuid_msgs nav_msgs
)

set(osqp_INCLUDE_DIRS /usr/local/include/osqp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${osqp_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/optim/b_spline_optim.cpp
  src/optim/qp_optim.cpp 
  src/planner/lattice_planner.cpp 
  src/controller/controller_node.cpp 
  src/controller/controller.cpp 
  src/collision_detection.cpp 
  src/m_common.cpp 
  src/reference_line.cpp 
  src/visualization.cpp 
  src/plan.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(plan src/main.cpp)

add_dependencies(plan 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(plan 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  OsqpEigen  
)

add_executable(controller src/controller/controller_main.cpp)

add_dependencies(controller 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(controller 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  OsqpEigen 
)

# install(TARGETS plan
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  
# )