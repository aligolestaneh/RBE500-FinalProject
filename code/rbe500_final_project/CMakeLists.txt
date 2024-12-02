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
find_package(sensor_msgs REQUIRED)
find_package(rbe500_final_project_msgs REQUIRED)
find_package(rbe500_final_project_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(open_manipulator_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

#Include EIgen lib
find_package(Eigen3 REQUIRED)

# setup targets
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)

################################################################################
# Build Libraries
################################################################################
#Core library independent of ROS interface
add_library(manipulator_core_lib
  "src/manipulator_core.cpp"
)
set(DEPENDENCIES
  "std_msgs"
  "geometry_msgs"
  "rclcpp"
  "visualization_msgs"
)
target_link_libraries(manipulator_core_lib)
ament_target_dependencies(manipulator_core_lib ${DEPENDENCIES} Eigen3)

add_library(manipulator_ik_ros_lib
  "src/manipulator_ik_ros.cpp"
)
set(DEPENDENCIES
  "rbe500_final_project_msgs"
  "std_msgs"
  "geometry_msgs"
  "rclcpp"
  "visualization_msgs"
)
target_link_libraries(manipulator_ik_ros_lib manipulator_core_lib)
ament_target_dependencies(manipulator_ik_ros_lib ${DEPENDENCIES})

##Forward kinematics lib
add_library(manipulator_fk_ros_lib
  "src/manipulator_fk_ros.cpp"
)
set(DEPENDENCIES
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "rclcpp"
  "visualization_msgs"
)
target_link_libraries(manipulator_fk_ros_lib manipulator_core_lib)
ament_target_dependencies(manipulator_fk_ros_lib ${DEPENDENCIES})

##Follow action lib
add_library(manipulator_follow_actions_lib
  "src/manipulator_follow_actions.cpp"
)
set(DEPENDENCIES
  "open_manipulator_msgs"
  "rbe500_final_project_msgs"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "rclcpp"
  "visualization_msgs"
)
target_link_libraries(manipulator_follow_actions_lib ${YAML_CPP_LIBRARIES})
ament_target_dependencies(manipulator_follow_actions_lib ${DEPENDENCIES})

##Joint Position UPdater lib
add_library(manipulator_jp_updater_ros_lib
  "src/manipulator_jp_updater_ros.cpp"
)
set(DEPENDENCIES
  "open_manipulator_msgs"
  "rbe500_final_project_msgs"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "rclcpp"
  "visualization_msgs"
)
target_link_libraries(manipulator_jp_updater_ros_lib ${YAML_CPP_LIBRARIES})
ament_target_dependencies(manipulator_jp_updater_ros_lib ${DEPENDENCIES})

##Velocity Kinematics lib
add_library(manipulator_vlk_ros_lib
  "src/manipulator_vlk_ros.cpp"
)
set(DEPENDENCIES
  "open_manipulator_msgs"
  "rbe500_final_project_msgs"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "rclcpp"
  "visualization_msgs"
)
target_link_libraries(manipulator_vlk_ros_lib ${YAML_CPP_LIBRARIES})
ament_target_dependencies(manipulator_vlk_ros_lib ${DEPENDENCIES})
################################################################################
# Build Executables
################################################################################
#ROS Node for Mnaipulator
add_executable(manipulator_ik_node 
  src/manipulator_ik_node.cpp
)
target_link_libraries(manipulator_ik_node manipulator_ik_ros_lib)
ament_target_dependencies(manipulator_ik_node ${DEPENDENCIES})

#ROS Node for FK Mnaipulator
add_executable(manipulator_fk_node 
  src/manipulator_fk_node.cpp
)
target_link_libraries(manipulator_fk_node manipulator_fk_ros_lib)
ament_target_dependencies(manipulator_fk_node ${DEPENDENCIES})

#ROS Node for Follow Mnaipulator
add_executable(manipulator_follow_actions_node 
  src/manipulator_follow_actions_node.cpp
)
target_link_libraries(manipulator_follow_actions_node manipulator_follow_actions_lib)
ament_target_dependencies(manipulator_follow_actions_node ${DEPENDENCIES})

#ROS Node for Joint Position updater
add_executable(manipulator_jp_updater_node 
  src/manipulator_jp_updater_node.cpp
)
target_link_libraries(manipulator_jp_updater_node manipulator_jp_updater_ros_lib)
ament_target_dependencies(manipulator_jp_updater_node ${DEPENDENCIES})

################################################################################
# Install
################################################################################
install(TARGETS manipulator_ik_node manipulator_fk_node manipulator_follow_actions_node
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
