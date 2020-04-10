cmake_minimum_required(VERSION 3.5)
project(hacked_demo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(listener64 src/listener64.cpp)
ament_target_dependencies(listener64 rclcpp std_msgs)

add_executable(talker64 src/talker64.cpp)
ament_target_dependencies(talker64 rclcpp std_msgs)

add_executable(roundtrip64 src/roundtrip64.cpp)
ament_target_dependencies(roundtrip64 rclcpp std_msgs)

add_executable(selftalker64 src/selftalker64.cpp)
ament_target_dependencies(selftalker64 rclcpp std_msgs)

install(TARGETS
  listener64
  talker64
  roundtrip64
  selftalker64
  DESTINATION lib/${PROJECT_NAME})

ament_package()
