cmake_minimum_required(VERSION 3.8)
project(mushr_sim)

# Helpful warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core build tools
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Interface generation (srv/msg)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# If you have Python modules under mushr_sim/ (a Python package),
# this installs them into the ROS 2 install space.
ament_python_install_package(${PROJECT_NAME})

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CarPose.srv"
  DEPENDENCIES std_msgs
)

# Install Python executable scripts (ROS 2 convention: lib/<pkg>)
install(PROGRAMS
  scripts/clicked_point_to_reposition
  scripts/fake_vesc_driver_node
  scripts/keyboard_teleop
  scripts/mushr_sim
  DESTINATION lib/${PROJECT_NAME}
)

# Install shared resources
install(DIRECTORY
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

# Export runtime dependency for generated interfaces
ament_export_dependencies(rosidl_default_runtime)

ament_package()