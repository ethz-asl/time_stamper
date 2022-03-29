cmake_minimum_required(VERSION 3.0.2)
project(time_stamper_ptp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin_simple REQUIRED)

catkin_simple()

#cs_add_library(${PROJECT_NAME})

cs_add_executable(${PROJECT_NAME}-node
        src/Main.cpp
        )

cs_install()
cs_export()