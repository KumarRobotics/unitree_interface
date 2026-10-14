cmake_minimum_required(VERSION 3.8)
project(unitree_teleop_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(unitree_teleop_input_adapter SHARED
  src/input_adapter.cpp
)
target_compile_features(unitree_teleop_input_adapter PUBLIC cxx_std_17)
target_include_directories(unitree_teleop_input_adapter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(unitree_teleop_input_adapter
  mavros_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS unitree_teleop_input_adapter
  EXPORT export_unitree_teleop_common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_unitree_teleop_common HAS_LIBRARY_TARGET)
ament_export_dependencies(mavros_msgs rclcpp sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
