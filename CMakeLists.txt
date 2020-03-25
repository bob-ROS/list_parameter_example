cmake_minimum_required(VERSION 2.8.3)
project(list_parameter_example)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#add_definitions(${PCL_DEFINITIONS}) #fixes GICP according to forum
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package (Eigen3 REQUIRED NO_MODULE)

include_directories(
  include
  ${EXTERNAL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/example.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
