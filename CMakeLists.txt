cmake_minimum_required(VERSION 2.8.3)
project(binary_logger)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -Dlinux")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  nodelet
  tf
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES binary_logger
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs tf
  #DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(binary_logger
  src/${PROJECT_NAME}/joint_state_logger.cpp
  src/${PROJECT_NAME}/pose_stamped_logger.cpp
  src/${PROJECT_NAME}/wrench_stamped_logger.cpp
  src/${PROJECT_NAME}/twist_stamped_logger.cpp
  src/${PROJECT_NAME}/float64_multiarray_logger.cpp
  src/${PROJECT_NAME}/float64_logger.cpp
  src/${PROJECT_NAME}/imu_logger.cpp
  src/${PROJECT_NAME}/tf_logger.cpp
)

add_dependencies(binary_logger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(binary_logger
  ${catkin_LIBRARIES}
)

add_executable(logger_start src/logger_start_node.cpp)
add_dependencies(logger_start  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(logger_start
  ${catkin_LIBRARIES}
)


add_executable(automatic_logger_node src/automatic_logger_node.cpp)
add_dependencies(automatic_logger_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(automatic_logger_node
  ${catkin_LIBRARIES}
)
