cmake_minimum_required(VERSION 2.8.3)
project(obstacle_imaginer)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  #rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
  pcl_ros
  pcl_conversions
  tflib
  #vislib
  #pluginlib
  core_trajectory_msgs
  #tic_toc
  message_filters
  #cv_bridge
  #templib
)


find_package(base)
find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${base_INCLUDE_DIRS}
  include
)

add_executable(obstacle_imaginer src/obstacle_imaginer.cpp)

add_dependencies(obstacle_imaginer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${base_EXPORTED_TARGETS})

target_link_libraries(obstacle_imaginer
  ${catkin_LIBRARIES}
  ${base_LIBRARIES}
  ${PCL_LIBRARIES}
  "/usr/lib/x86_64-linux-gnu/libOpenCL.so.1.0.0"
)