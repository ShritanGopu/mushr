cmake_minimum_required(VERSION 3.8)
project(mushr_base)

# Core build tools
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Typical for Python nodes using messages
find_package(rclpy REQUIRED)

# Custom message package dependency (replace with your actual package)
find_package(vesc REQUIRED)

# Install the Python package (if you have a setup.py / src/mushr_base/ tree)
ament_python_install_package(${PROJECT_NAME})

# Install Python executables (scripts)
install(
  PROGRAMS
    scripts/racecar_state
    scripts/joy_teleop
    scripts/nav_msg_converter
  DESTINATION lib/${PROJECT_NAME}
)

# Install non-code resources (config, launch, maps)
install(
  DIRECTORY
    config
    launch
    maps
  DESTINATION share/${PROJECT_NAME}
)

# Export package to the ament index
ament_package()
