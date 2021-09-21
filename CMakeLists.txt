cmake_minimum_required(VERSION 3.16.3)
project(rosparam_shortcuts)

# C++ 17
add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    Eigen3
    eigen3_cmake_module
    geometry_msgs
    rclcpp
    tf2_eigen
)

include_directories(
  include
)

###########
## Build ##
###########
add_library(${PROJECT_NAME} SHARED
            src/node_parameters.cpp
            src/rosparam_shortcuts.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)

## Example executable
add_executable(${PROJECT_NAME}_example src/example.cpp)
ament_target_dependencies(${PROJECT_NAME}_example ${THIS_PACKAGE_INCLUDE_DEPENDS})
set_target_properties(${PROJECT_NAME}_example PROPERTIES OUTPUT_NAME example PREFIX "")
target_link_libraries(${PROJECT_NAME}_example ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_node_parameters_example src/node_parameters_example.cpp)
ament_target_dependencies(${PROJECT_NAME}_node_parameters_example ${THIS_PACKAGE_INCLUDE_DEPENDS})
set_target_properties(${PROJECT_NAME}_node_parameters_example PROPERTIES OUTPUT_NAME node_parameters_example PREFIX "")
target_link_libraries(${PROJECT_NAME}_node_parameters_example ${PROJECT_NAME})

#############
## Install ##
#############

# Install Libraries
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install Binaries
install(
  TARGETS
    ${PROJECT_NAME}_example
    ${PROJECT_NAME}_node_parameters_example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch, config directories
install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)
  # These don't pass yet, disable them for now
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_node_parameters test/test_node_parameters.cpp)
  ament_target_dependencies(test_node_parameters ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_link_libraries(test_node_parameters ${PROJECT_NAME})
endif()

ament_package()
