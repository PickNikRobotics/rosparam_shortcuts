cmake_minimum_required(VERSION 2.8.3)
project(rosparam_shortcuts)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  eigen_conversions # TODO(davetcoleman) remove this dependency, only for build testing
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS Eigen
)

include_directories(SYSTEM
  ${EIGEN_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/rosparam_shortcuts.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Install libraries
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)