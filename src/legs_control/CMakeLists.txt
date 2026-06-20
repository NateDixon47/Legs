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

add_executable(ik_node src/ik_node.cpp)
add_executable(step src/step.cpp)

ament_target_dependencies(ik_node
  rclcpp
  utilities
  std_msgs
  sensor_msgs)

# Lets #include "legs_control/..." headers resolve from this package's include/.
target_include_directories(step PRIVATE include)
ament_target_dependencies(step
  rclcpp
  utilities
  std_msgs
  sensor_msgs)

install(TARGETS ik_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS step
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
