cmake_minimum_required(VERSION 3.0.2)
project(fake_odom)
add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ros_node_utils
  ros_transform_utils
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    # ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp
    ros_node_utils
    ros_transform_utils
)

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fake_odom_node src/fake_odom_node.cpp
  src/fake_odom/FakeOdometry.cpp
  
)
target_link_libraries(fake_odom_node
  ${catkin_LIBRARIES}
)