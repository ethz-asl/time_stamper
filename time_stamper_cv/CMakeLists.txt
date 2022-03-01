cmake_minimum_required(VERSION 3.0.2)
project(time_stamper_cv)

find_package(catkin_simple REQUIRED)

catkin_simple()

#cs_add_library(${PROJECT_NAME})
cs_add_executable(${PROJECT_NAME}-node src/Main.cpp)


#target_link_libraries(
#        ${PROJECT_NAME}-node
#        ${PROJECT_NAME}
#)

cs_install()
cs_export()