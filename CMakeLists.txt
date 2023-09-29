cmake_minimum_required(VERSION 3.12)
project(robotarm_interface)

set(CMAKE_CXX_STANDARD 11)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

# required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Boost REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)


# create executable
add_executable(interface
  src/main.cpp
  src/RosInterface.hpp
  src/RosInterface.cpp
  src/LowLevelDriver.hpp
  src/LowLevelDriver.cpp
  src/Position.hpp
  src/Position.cpp
)

ament_target_dependencies(interface rclcpp rclcpp_action Boost)

# set message files
set(action_files
  "action/Position.action"
  "action/Servo.action"
)

# generate message files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
)

ament_export_dependencies(rosidl_default_runtime)

# install application
install(TARGETS
interface
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(interface "${cpp_typesupport_target}")
target_link_libraries(interface Threads::Threads)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()