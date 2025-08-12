cmake_minimum_required(VERSION 3.8)
project(traversal2)

# Compiler settings for ROS2
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies - ROS2 style
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages - ROS2 style
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelRpm.msg"
  DEPENDENCIES sensor_msgs std_msgs
)

###################################
## ament specific configuration ##
###################################
# Export dependencies for other packages
ament_export_dependencies(rosidl_default_runtime)

###########
## Build ##
###########

# Include directories
include_directories(include)

## Declare a C++ library (uncomment if needed)
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/traversal2.cpp
# )

## Add dependencies for the library
# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp
#   sensor_msgs
#   std_msgs
# )

## Declare a C++ executable (uncomment if needed)
# add_executable(${PROJECT_NAME}_node src/traversal2_node.cpp)

## Add dependencies for the executable
# ament_target_dependencies(${PROJECT_NAME}_node
#   rclcpp
#   sensor_msgs
#   std_msgs
# )

## Link custom message interfaces to executable
# rosidl_target_interfaces(${PROJECT_NAME}_node
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )

#############
## Install ##
#############

## Install executables
# install(TARGETS ${PROJECT_NAME}_node
#   DESTINATION lib/${PROJECT_NAME}
# )

## Install Python scripts
# install(PROGRAMS
#   scripts/my_python_script.py
#   DESTINATION lib/${PROJECT_NAME}
# )

## Install launch files
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

## Install other files (config, etc.)
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}
# )

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  ## Add C++ tests (uncomment if needed)
  # find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(${PROJECT_NAME}-test test/test_traversal2.cpp)
  # if(TARGET ${PROJECT_NAME}-test)
  #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  #   ament_target_dependencies(${PROJECT_NAME}-test rclcpp)
  # endif()
endif()

# Must be last
ament_package()
