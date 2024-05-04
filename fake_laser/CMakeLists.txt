cmake_minimum_required(VERSION 3.0.2)
project(fake_laser)
add_compile_options(-std=c++17)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_core
  grid_map_ros
  grid_map_cv
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    # ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp
    grid_map_core
    grid_map_ros
    grid_map_cv
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

add_executable(fake_laser_node 
  src/fake_laser/fake_laser_node.cpp
  src/fake_laser/FakeLaser.cpp
  src/fake_laser/LaserGenerator.cpp
  src/occupancyMap/OccupancyGridMap.cpp
  src/occupancyMap/OccupancyGridMapHelper.cpp
  src/occupancyMap/OccupancyGridMapRosConverter.cpp
  
)
target_link_libraries(fake_laser_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
