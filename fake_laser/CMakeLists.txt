cmake_minimum_required(VERSION 3.0.2)
project(fake_laser)
add_compile_options(-std=c++14)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ros_node_utils
  tf2_ros
  ros_transform_utils
  pcl_conversions
  ros_pcl_utils
  occupancy_grid_map
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    # ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp
    ros_node_utils
    tf2_ros
    ros_transform_utils
    pcl_conversions
    ros_pcl_utils
    occupancy_grid_map
)

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(fake_laser_node src/fake_laser_node.cpp
  src/fake_laser/FakeLaser.cpp
  src/fake_laser/LaserGenerator.cpp
  
)
target_link_libraries(fake_laser_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)