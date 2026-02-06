cmake_minimum_required(VERSION 3.8)
project(unitree_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Find unitree_sdk2
find_package(unitree_sdk2 REQUIRED
  HINTS /opt/unitree_robotics/lib/cmake/unitree_sdk2
)

# Include directories
include_directories(include)

# Create executable
add_executable(unitree_teleop
  src/unitree_teleop.cpp
)

ament_target_dependencies(unitree_teleop
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

target_link_libraries(unitree_teleop unitree_sdk2)

# Install executable
install(TARGETS
  unitree_teleop
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
