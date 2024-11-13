cmake_minimum_required(VERSION 3.8)
project(rbe500_final_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

#Include EIgen lib
find_package(Eigen3 REQUIRED)

# setup targets
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

################################################################################
# Build Libraries
################################################################################
#Core library independent of ROS interface
add_library(manipulator_core_lib
  "src/manipulator_core.cpp"
)
# set(DEPENDENCIES
#   "geometry_msgs"
#   "rclcpp"
# )
target_link_libraries(manipulator_core_lib)
ament_target_dependencies(manipulator_core_lib Eigen3)

add_library(manipulator_ros_lib
  "src/manipulator_ros.cpp"
)
set(DEPENDENCIES
  "geometry_msgs"
  "rclcpp"
)
target_link_libraries(manipulator_ros_lib manipulator_core_lib)
ament_target_dependencies(manipulator_ros_lib ${DEPENDENCIES})

################################################################################
# Build Executables
################################################################################
#ROS Node for Mnaipulator
add_executable(manipulator_node 
  src/manipulator_node.cpp
)
target_link_libraries(manipulator_node manipulator_ros_lib)
ament_target_dependencies(manipulator_node ${DEPENDENCIES})


################################################################################
# Install
################################################################################
install(TARGETS manipulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  ################################################################################
  # TEST Cases
  ################################################################################
  set(TESTFILES
  test/main.cpp
  test/manipulator_core_test.cpp
  test/basic_eigen_test.cpp)

  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  target_link_libraries(${PROJECT_NAME}_test manipulator_core_lib)
  
  install(TARGETS
  ${PROJECT_NAME}_test
  DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
