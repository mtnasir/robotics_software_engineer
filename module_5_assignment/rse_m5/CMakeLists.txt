cmake_minimum_required(VERSION 3.8)
project(rse_m5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(EIGEN_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../../extern/eigen)
find_package(Eigen3 REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(angles REQUIRED)

find_package(visualization_msgs REQUIRED)

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

add_executable(velocity_controller_drive src/velocity_controller_drive.cpp)
ament_target_dependencies(velocity_controller_drive rclcpp tf2_ros geometry_msgs)

add_executable(goal_linear src/goal_linear.cpp)
ament_target_dependencies(goal_linear rclcpp tf2_ros geometry_msgs nav_msgs)

add_executable(goal_planer src/goal_planer.cpp)
ament_target_dependencies(goal_planer rclcpp tf2_ros geometry_msgs nav_msgs)

add_library(lqr_lib STATIC src/lqr_lib.cpp)
target_include_directories(lqr_lib PRIVATE ${EIGEN_INCLUDE_DIR} include)
target_link_libraries(lqr_lib Eigen3::Eigen)

add_executable(lqr_node src/lqr_node.cpp)
target_link_libraries(lqr_node lqr_lib Eigen3::Eigen)
ament_target_dependencies(lqr_node rclcpp nav_msgs geometry_msgs tf2 angles visualization_msgs)
target_include_directories(lqr_node PRIVATE ${EIGEN_INCLUDE_DIR} include)

add_executable(task2_lqr src/task2_lqr.cpp)
target_link_libraries(task2_lqr lqr_lib Eigen3::Eigen)
ament_target_dependencies(task2_lqr rclcpp nav_msgs geometry_msgs tf2 angles visualization_msgs)
target_include_directories(task2_lqr PRIVATE ${EIGEN_INCLUDE_DIR} include)

# Add executable
add_executable(non src/non.cpp)
target_link_libraries(non Eigen3::Eigen)
ament_target_dependencies(non rclcpp nav_msgs geometry_msgs tf2 angles visualization_msgs)

ament_target_dependencies(non
  rclcpp
  Eigen3
  tf2
  tf2_ros
  angles
  geometry_msgs
  nav_msgs
  visualization_msgs
)
target_include_directories(non PRIVATE ${EIGEN_INCLUDE_DIR} include)


install(TARGETS 
maze_control 
line_following
non
task1 
task2_lqr
task3
velocity_controller_drive 
goal_linear
goal_planer
lqr_lib
lqr_node

DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf rviz worlds models config

DESTINATION share/${PROJECT_NAME}
)

ament_package()


# 
