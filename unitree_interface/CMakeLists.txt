cmake_minimum_required(VERSION 3.8)
project(unitree_interface)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install example launch files as executables (optional)
ament_package()