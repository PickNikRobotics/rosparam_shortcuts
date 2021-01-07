cmake_minimum_required(VERSION 3.5)
project(rosparam_shortcuts)

# C++ 17
add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

###########
## Build ##
###########

add_library(${PROJECT_NAME} SHARED
            src/rosparam_shortcuts.cpp)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}
    geometry_msgs
    rclcpp
    tf2_eigen
)
## Example executable
add_executable(${PROJECT_NAME}_example src/example.cpp)
set_target_properties(${PROJECT_NAME}_example PROPERTIES OUTPUT_NAME example PREFIX "")
target_link_libraries(${PROJECT_NAME}_example
  ${PROJECT_NAME}
)

#############
## Install ##
#############

# Mark cpp header files for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark roslaunch files for installation
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
  PATTERN ".svn" EXCLUDE
)

# Mark config files for installation
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
  PATTERN ".svn" EXCLUDE
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(Eigen3)
ament_export_dependencies(rclcpp)
ament_export_dependencies(tf2_eigen)
ament_export_dependencies(geometry_msgs)
ament_package()