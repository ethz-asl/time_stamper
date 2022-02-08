cmake_minimum_required(VERSION 3.0.2)
project(time_stamper_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        roscpp
        std_msgs
        message_generation
        )

add_message_files(FILES
        Timestamp.msg
        )

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES rpi_timestamp_generator
        CATKIN_DEPENDS roscpp message_runtime
)

set(CMAKE_MODULE_PATH
        ${CMAKE_MODULE_PATH}
        ${PROJECT_SOURCE_DIR}/cmake/
        )

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/SysfsPwm.cpp
        src/Node.cpp
        src/TimestampManager.cpp include/TimestampManager.h)

add_executable(${PROJECT_NAME}-node
        src/Main.cpp
        )

#find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
#target_link_libraries(${PROJECT_NAME}_node ${WIRINGPI_LIBRARIES})
target_link_libraries(
        ${PROJECT_NAME}-node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}

)