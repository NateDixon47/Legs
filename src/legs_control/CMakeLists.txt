cmake_minimum_required(VERSION 3.8)
project(legs_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(utilities REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(legs_dynamics REQUIRED)

add_executable(ik_node src/ik_node.cpp)
add_executable(torque_node src/torque_node.cpp)
add_executable(controller_node src/controller_node.cpp)
add_executable(traj_node src/traj_node.cpp)
add_executable(inverse_dynamics src/inverse_dynamics.cpp)


# trajectory_generator.hpp lives in THIS package's include/, so it needs an include
# path, not a find_package -- a package cannot depend on itself.
target_include_directories(traj_node PRIVATE include)
ament_target_dependencies(traj_node
  rclcpp
  utilities
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  legs_dynamics)


ament_target_dependencies(inverse_dynamics
  rclcpp
  utilities
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  legs_dynamics)


ament_target_dependencies(ik_node
  rclcpp
  utilities
  std_msgs
  sensor_msgs)


ament_target_dependencies(torque_node
  rclcpp
  utilities
  std_msgs
  sensor_msgs
  nav_msgs
  legs_dynamics)


  ament_target_dependencies(controller_node
  rclcpp
  utilities
  std_msgs
  sensor_msgs
  nav_msgs
  legs_dynamics)

install(TARGETS ik_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS torque_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS traj_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS inverse_dynamics
  DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
