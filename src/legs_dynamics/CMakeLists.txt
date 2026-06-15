cmake_minimum_required(VERSION 3.8)
project(legs_dynamics)

# Emit compile_commands.json so clangd/IDE can resolve includes (e.g. MuJoCo).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(utilities REQUIRED)
find_package(mujoco_vendor REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(robot_model src/robot_model.cpp)

target_include_directories(robot_model PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

target_link_libraries(robot_model PUBLIC mujoco::mujoco)

ament_target_dependencies(robot_model PUBLIC utilities Eigen3)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS robot_model
  EXPORT export_robot_model
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_robot_model HAS_LIBRARY_TARGET)
ament_export_dependencies(utilities mujoco_vendor Eigen3)

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

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_robot_model test/test_robot_model.cpp)
  target_link_libraries(test_robot_model robot_model)
endif()

ament_package()
