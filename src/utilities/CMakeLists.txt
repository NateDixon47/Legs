cmake_minimum_required(VERSION 3.8)
project(utilities)

# C++17 for std::optional / structured bindings (auto [a, b] = ...)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# --- shared kinematics library (pure C++ / Eigen, no rclcpp) ---
add_library(kinematics src/transforms.cpp src/leg.cpp)
target_include_directories(kinematics PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(kinematics Eigen3::Eigen)

# --- executables (each links the shared lib so it sees form_TF_fixed) ---
add_executable(fk src/fk.cpp)
target_link_libraries(fk kinematics)

add_executable(ik src/ik.cpp)
target_link_libraries(ik kinematics)

# --- install ---
install(DIRECTORY include/ DESTINATION include)
install(TARGETS kinematics
  EXPORT export_kinematics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(TARGETS fk ik
  DESTINATION lib/${PROJECT_NAME})

# export the library so downstream packages (e.g. bipedal_control) can link it
ament_export_targets(export_kinematics HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3)

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
