cmake_minimum_required(VERSION 3.8)
project(b2_native_rc_interface)

# ============================================================================
# Compiler Settings
# ============================================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 Standard (required for Unitree SDK2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# Find Dependencies
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Unitree ROS2 Messages (from unitree_ros2 package)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)

# ============================================================================
# Generate Custom Messages
# ============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NativeRC.msg"
  "msg/JoystickState.msg"
  "msg/ButtonState.msg"
  "msg/RobotState.msg"
  DEPENDENCIES 
    std_msgs
    builtin_interfaces
)

# Export message targets for downstream packages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# ============================================================================
# Include Directories
# ============================================================================
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# ============================================================================
# Libraries
# ============================================================================

# Key Parser Library
add_library(key_parser SHARED
  src/utils/key_parser.cpp
)
ament_target_dependencies(key_parser
  rclcpp
)

# Joystick Processor Library
add_library(joystick_processor SHARED
  src/utils/joystick_processor.cpp
)
ament_target_dependencies(joystick_processor
  rclcpp
)

# ============================================================================
# Executables
# ============================================================================

# Main Bridge Node
add_executable(rc_bridge_node
  src/nodes/rc_bridge_node.cpp
  src/bridge/rc_signal_bridge.cpp
)
ament_target_dependencies(rc_bridge_node
  rclcpp
  std_msgs
  unitree_go
  unitree_api
)
target_link_libraries(rc_bridge_node
  key_parser
  joystick_processor
  "${cpp_typesupport_target}"
)

# Test Subscriber Node (for debugging)
add_executable(rc_test_subscriber
  src/nodes/rc_test_subscriber.cpp
)
ament_target_dependencies(rc_test_subscriber
  rclcpp
  std_msgs
)
target_link_libraries(rc_test_subscriber
  "${cpp_typesupport_target}"
)

# RC Simulator Node (for local testing without B2 hardware)
add_executable(rc_simulator
  src/nodes/rc_simulator_node.cpp
)
ament_target_dependencies(rc_simulator
  rclcpp
  unitree_go
)
target_include_directories(rc_simulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ============================================================================
# Install
# ============================================================================

# Install libraries
install(TARGETS
  key_parser
  joystick_processor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  rc_bridge_node
  rc_test_subscriber
  rc_simulator
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ============================================================================
# Testing
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_key_parser test/test_key_parser.cpp)
  target_link_libraries(test_key_parser key_parser)
endif()

# ============================================================================
# Ament Package
# ============================================================================
ament_export_include_directories(include)
ament_export_libraries(key_parser joystick_processor)
ament_export_dependencies(
  rclcpp
  std_msgs
  unitree_go
  unitree_api
  rosidl_default_runtime
)

ament_package()
