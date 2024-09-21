cmake_minimum_required(VERSION 3.8)
project(rse_m4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(maze_control src/maze_control.cpp)
ament_target_dependencies(maze_control rclcpp sensor_msgs geometry_msgs cv_bridge)
target_link_libraries(maze_control ${OpenCV_LIBS})


add_executable(task3 src/task3.cpp)
ament_target_dependencies(task3 rclcpp sensor_msgs geometry_msgs cv_bridge)
target_link_libraries(task3 ${OpenCV_LIBS})


add_executable(line_following src/line_following.cpp)
ament_target_dependencies(line_following rclcpp sensor_msgs geometry_msgs cv_bridge)
target_link_libraries(line_following ${OpenCV_LIBS})

add_executable(task1 src/task1.cpp)
ament_target_dependencies(task1 rclcpp sensor_msgs geometry_msgs cv_bridge)
target_link_libraries(task1 ${OpenCV_LIBS})


install(TARGETS maze_control line_following task1 task3

DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf rviz worlds models
DESTINATION share/${PROJECT_NAME}
)

ament_package()
