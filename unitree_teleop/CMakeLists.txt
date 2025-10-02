cmake_minimum_required(VERSION 3.8)
project(unitree_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)

# Include directories
include_directories(include)

# Create executable
add_executable(unitree_teleop 
  src/unitree_teleop.cpp
  src/common/ros2_sport_client.cpp
)

target_compile_features(unitree_teleop PRIVATE cxx_std_20)

ament_target_dependencies(unitree_teleop
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  unitree_api
  unitree_go
)

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