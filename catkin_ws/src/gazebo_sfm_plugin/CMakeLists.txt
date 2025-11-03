cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sfm_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)

# Add the include directory for lightsfm
include_directories(include)
include_directories(SYSTEM
  /home/alien/Desktop/catkin_ws/src/lightsfm/  # Path to lightsfm headers (sfm.hpp)
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES PedestrianSFMPlugin
  CATKIN_DEPENDS gazebo_ros roscpp
)

add_library(PedestrianSFMPlugin src/PedestrianSFMPlugin.cpp)


# Link the required Gazebo and Catkin libraries
target_link_libraries(PedestrianSFMPlugin 
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
)




install(TARGETS
  PedestrianSFMPlugin
  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

