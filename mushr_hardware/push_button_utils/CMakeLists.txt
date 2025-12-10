cmake_minimum_required(VERSION 3.5)
project(push_button_utils)

## Compile as C++14, supported in ROS 2 Foxy and newer
# add_compile_options(-std=c++14)

############################################################
## Find ament and ROS 2 dependencies                      ##
############################################################

## In ROS 2, packages are found using ament. If your package
## depends on other ROS 2 packages, list them under
## find_package(... REQUIRED).
##
## For example, for C++ nodes using rclcpp:
##   find_package(rclcpp REQUIRED)
##
## rospy is not used in ROS 2 (Python nodes use rclpy instead).
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)

## System dependencies are found using standard CMake
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS 2 interface files (msg/srv/action)
################################################

## In ROS 2, messages/services/actions are generated with rosidl.
## To use this:
## * In package.xml:
##     <build_depend>rosidl_default_generators</build_depend>
##     <exec_depend>rosidl_default_runtime</exec_depend>
## * Uncomment rosidl_generate_interfaces(...) below
## * List your .msg / .srv / .action files in the appropriate lists

## Example:
# set(MSG_FILES
#   "msg/Message1.msg"
#   "msg/Message2.msg"
# )

# set(SRV_FILES
#   "srv/Service1.srv"
# )

# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${MSG_FILES}
#   ${SRV_FILES}
#   DEPENDENCIES std_msgs
# )

################################################
## Declare ROS 2 parameters / configuration    ##
################################################

## ROS 2 does not use dynamic_reconfigure.
## To expose parameters dynamically:
##   * Use declare_parameter() in your node
##   * Register a parameter callback if needed
##
## YAML config files can be installed under share/ and loaded via launch files.

###################################
## ament package configuration    ##
###################################

## ament_package() exports this package to downstream packages.
## You may list:
##   INCLUDE_DIRS – public headers
##   LIBRARIES – libraries this package builds
##   DEPENDS – non-ROS system libraries
ament_package(
  # INCLUDE_DIRS include
  # LIBRARIES push_button_utils
  # DEPENDS Boost
)

###########
## Build ##
###########

## Specify include directories
include_directories(
  # include
)

## Declare a C++ executable
## In ROS 2, each target is built independently, not in a merged workspace.
## Example:
# add_executable(push_button_utils_node src/push_button_utils_node.cpp)

## Link against ament / rclcpp libraries
# ament_target_dependencies(push_button_utils_node
#   rclcpp
#   std_msgs
# )

#############
## Install ##
#############

## Executables should be installed into lib/<package_name>
# install(TARGETS push_button_utils_node
#   DESTINATION lib/${PROJECT_NAME}
# )

## Python scripts (if any)
# install(PROGRAMS
#   scripts/my_python_script.py
#   DESTINATION lib/${PROJECT_NAME}
# )

## Install header files
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
# )

## Install configuration or launch files
# install(FILES
#   launch/example.launch.py
#   config/example.yaml
#   DESTINATION share/${PROJECT_NAME}
# )

#############
## Testing ##
#############

## ROS 2 uses ament_cmake_gtest for C++ tests.
# find_package(ament_cmake_gtest REQUIRED)
# ament_add_gtest(${PROJECT_NAME}_test test/test_push_button_utils.cpp)
# if(TARGET ${PROJECT_NAME}_test)
#   target_link_libraries(${PROJECT_NAME}_test push_button_utils)
# endif()

ament_package()
