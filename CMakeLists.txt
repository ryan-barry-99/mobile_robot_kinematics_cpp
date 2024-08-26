cmake_minimum_required(VERSION 3.10)

# Set the project name
project(MobileRobotKinematics)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)

# Specify the Eigen include directory
include_directories(/opt/homebrew/include/eigen3)

# Add the source files
add_executable(mobile_robot_kinematics
    src/main.cpp
    src/MobileRobotKinematics.cpp
    src/Wheel.cpp
    src/MobileRobot.cpp
)

# Include directories for your own headers
target_include_directories(mobile_robot_kinematics PRIVATE ${CMAKE_SOURCE_DIR}/include)
