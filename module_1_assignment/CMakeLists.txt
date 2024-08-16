cmake_minimum_required(VERSION 3.10)
project(RobotProject)

set(CMAKE_CXX_STANDARD 17)


# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

# Create executables
add_executable(task1 src/Task1.cpp)
add_executable(task2 src/Task2.cpp)
add_executable(task3 src/Task3.cpp lib/sensor.cpp)  # Include sensor.cpp here

# Link libraries if needed
target_include_directories(task3 PUBLIC ${PROJECT_SOURCE_DIR}/include)
# No need to link 'sensor' unless it's a separate library

# Print project source directory for debugging
message(STATUS "Project source directory: ${PROJECT_SOURCE_DIR}")
