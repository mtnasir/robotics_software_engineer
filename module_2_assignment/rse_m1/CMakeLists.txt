cmake_minimum_required(VERSION 3.8)
project(rse_m1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(rse1 src/rse1.cpp)
ament_target_dependencies(rse1 geometry_msgs rclcpp)

add_executable(turtle_drive src/turtle_drive.cpp)
ament_target_dependencies(turtle_drive geometry_msgs rclcpp)


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

message(PROJECT_NAME)
install(TARGETS
rse1
turtle_drive
DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})


ament_package()
